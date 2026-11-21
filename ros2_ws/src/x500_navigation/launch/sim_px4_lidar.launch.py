#!/usr/bin/env python3
"""Shared simulation stack: x500 with a 2D lidar, PX4 SITL, SLAM and Nav2.

Not meant to be launched directly; use a world launch file, which sets the
defaults for its world and includes this one:

    ros2 launch x500_navigation sim_baylands.launch.py   # outdoor park
    ros2 launch x500_navigation sim_office.launch.py     # indoor office

Starts, in this order:
  1. Gazebo Harmonic + PX4 SITL (gz_x500_lidar_2d).
     * world_sdf empty: a world shipped with PX4 (``world``), and PX4's own
       script starts Gazebo, as ``make px4_sitl`` does.
     * world_sdf set: this launch starts Gazebo with that file (PX4's script
       only looks in PX4's own worlds folder), waits until it is up, then
       starts PX4, which attaches to the running world.
     PX4 is told not to need a ground station (NAV_DLL_ACT=0), to stay in
     OFFBOARD without an RC link (COM_RCL_EXCEPT=4), and to hover (Hold) if
     OFFBOARD setpoints stop for 3 s (COM_OF_LOSS_T=3, COM_OBL_RC_ACT=5).
  2. MicroXRCE-DDS agent, which exposes PX4 as /fmu/* topics.
  3. gz_bridge (this repo): /clock and the lidar scan as /scan.
  4. TF from PX4 odometry: odom -> base_footprint (x, y, heading, at floor
     level) -> base_link (height, roll, pitch), plus /odom; and the fixed
     base_link -> link (the lidar frame).
  5. slam_toolbox (online async): builds /map and publishes map -> odom.
  6. Nav2 (navigation_launch.py).
  7. x500_px4_bridge: Nav2 /cmd_vel_nav -> PX4 OFFBOARD velocity setpoints.
     It arms, takes off to ``altitude`` and holds there.
  8. RViz. Click "2D Goal Pose" to fly somewhere.
  9. With explore:=true: explore_lite, started ``explore_delay`` s after the
     drone has taken off. It sends frontier after frontier to Nav2 until the
     map is complete (don't send your own goals meanwhile).
"""

import math
import os
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

MODEL = 'x500_lidar_2d'
TAG = '[sim]'

# PX4 and the Gazebo server/GUI, whoever started them. Anchored at the start
# of the command line, so a shell or editor that merely mentions these words
# in its own command line is never matched (pkill -f matches anywhere).
SIM_PROCESSES = (r'^\S*px4_sitl_default/bin/px4( |$)',
                 r'^gz sim .*-s \S*\.sdf',
                 r'^gz sim -g')

# Command-line fragments of the nodes this launch starts. Used to find copies
# left behind by a launch that crashed or was killed: they keep publishing TF
# with a frozen clock, which floods every node with TF_OLD_DATA warnings.
# Each entry: all of its fragments must appear in the command line.
OUR_NODES = (
    ('x500_px4_bridge/lib/x500_px4_bridge/',),       # gz_bridge, tf2_broadcaster, bridge
    ('async_slam_toolbox_node', 'x500_navigation/config/slam_'),
    ('/opt/ros/humble/lib/nav2_', '/tmp/launch_params_'),  # Nav2 started by a launch
    ('static_transform_publisher --x 0.12 --y 0 --z 0.315',),
    ('MicroXRCEAgent udp4 -p 8888',),
    ('rviz2', 'x500_navigation/rviz/sim_'),
    ('explore_lite/lib/explore_lite/explore',),
)
# Never treat these as our nodes, even if their command line mentions the
# fragments above (a terminal running `pgrep ...x500_px4_bridge...`, an editor).
NOT_NODES = {'bash', 'sh', 'dash', 'zsh', 'fish', 'grep', 'pgrep', 'pkill', 'ps',
             'vim', 'nvim', 'vi', 'nano', 'emacs', 'code', 'less', 'cat', 'tail', 'head',
             'watch', 'timeout', 'nohup', 'sudo', 'ssh', 'tmux', 'screen'}
# Any of the simulation launch files (sim_baylands, sim_office, ...).
THIS_LAUNCH = 'launch x500_navigation sim_'


def _flag(context, name):
    return context.launch_configurations[name].lower() in ('true', '1')


def _running(pattern):
    return subprocess.run(['pgrep', '-f', pattern], capture_output=True,
                          check=False).returncode == 0


def _kill_sim(grace_s=5.0):
    """SIGINT PX4 and Gazebo, then SIGKILL whatever is still alive after grace_s."""
    for pattern in SIM_PROCESSES:
        subprocess.run(['pkill', '-INT', '-f', pattern], check=False)
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline and any(_running(p) for p in SIM_PROCESSES):
        time.sleep(0.2)
    for pattern in SIM_PROCESSES:
        subprocess.run(['pkill', '-KILL', '-f', pattern], check=False)


_ABORTED = False


def _stop_px4_and_gazebo(context):
    if not _ABORTED:  # never touch the PX4 of the copy that is already running
        _kill_sim()
    return []


def _processes():
    """{pid: (ppid, cmdline)} for every process we can read."""
    procs = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
            with open(f'/proc/{entry}/stat') as f:
                ppid = int(f.read().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            continue
        procs[int(entry)] = (ppid, cmd)
    return procs


def _ancestors(pid, procs):
    seen = []
    while pid in procs and pid > 1 and pid not in seen:
        seen.append(pid)
        pid = procs[pid][0]
    return seen


def _preflight(context, actions):
    """Refuse to run twice; clear what a crashed run left behind; then start."""
    global _ABORTED
    procs = _processes()
    mine = set(_ancestors(os.getpid(), procs))
    others = [pid for pid, (_, cmd) in procs.items()
              if THIS_LAUNCH in cmd and 'ros2' in cmd and pid not in mine
              and not cmd.startswith(('bash', 'sh', '/bin/'))]
    if others:
        _ABORTED = True
        print(f'{TAG} ERROR: a simulation is already running (pid '
              f'{", ".join(map(str, others))}). Two copies fight over PX4, /clock and TF.\n'
              f'{TAG} Stop it first (Ctrl+C in its terminal, or '
              'pkill -INT -f "ros2 launch x500_navigation"), then launch again.')
        return [Shutdown(reason='simulation already running')]

    # Our nodes whose launch is gone (no `ros2 launch` among their parents).
    orphans = [pid for pid, (_, cmd) in procs.items()
               if pid not in mine and cmd
               and os.path.basename(cmd.split()[0]) not in NOT_NODES
               and any(all(f in cmd for f in k) for k in OUR_NODES)
               and not any('ros2 launch' in procs[a][1] for a in _ancestors(pid, procs))]
    if orphans:
        print(f'{TAG} stopping {len(orphans)} node(s) left over from an earlier run')
        for sig in ('-INT', '-KILL'):
            subprocess.run(['kill', sig, *map(str, orphans)], check=False,
                           capture_output=True)
            time.sleep(2.0)
    # A PX4 or Gazebo left over from a crashed run makes PX4 quit with
    # "PX4 server already running" (or attach to the wrong world).
    if any(_running(p) for p in SIM_PROCESSES):
        print(f'{TAG} stopping PX4/Gazebo left over from an earlier run')
        _kill_sim(grace_s=3.0)
    return actions


def _px4_params():
    return {
        'PX4_PARAM_NAV_DLL_ACT': '0',
        # If OFFBOARD setpoints stop (e.g. /clock gap while gz_bridge
        # restarts): wait 3 s, then hover in place (Hold). The default
        # needs an RC link, so without one PX4 falls back to Return/land.
        'PX4_PARAM_COM_OF_LOSS_T': '3.0',
        'PX4_PARAM_COM_OBL_RC_ACT': '5',
        'PX4_PARAM_COM_RCL_EXCEPT': '4',
    }


def _simulator(context):
    """Gazebo + PX4 SITL for the chosen world."""
    cfg = context.launch_configurations
    px4_dir = os.path.expanduser(cfg['px4_dir'])
    headless = _flag(context, 'headless')
    follow = _flag(context, 'follow')
    fx, fy, fz = (cfg['follow_offset'].split(',') + ['0', '0', '0'])[:3]
    env = {
        'PX4_GZ_MODEL_POSE': cfg['pose'],
        'PX4_GZ_NO_FOLLOW': '' if follow else '1',  # else PX4 locks the GUI camera
        'PX4_GZ_FOLLOW_OFFSET_X': fx,
        'PX4_GZ_FOLLOW_OFFSET_Y': fy,
        'PX4_GZ_FOLLOW_OFFSET_Z': fz,
        **_px4_params(),
    }
    make = ['make', 'px4_sitl', f'gz_{MODEL}']

    world_sdf = cfg['world_sdf']
    if not world_sdf:
        # A world from PX4's own folder: PX4's script starts Gazebo (and the GUI
        # unless HEADLESS is non-empty).
        env.update(PX4_GZ_WORLD=cfg['world'], HEADLESS='1' if headless else '')
        return [ExecuteProcess(cmd=make, cwd=px4_dir, additional_env=env, output='screen')]

    # Our own world file: start Gazebo with PX4's environment (model paths,
    # PX4 Gazebo plugins, server config). Once the world answers, start PX4:
    # its script finds the running world ("gazebo already running world")
    # and spawns the drone into it. (PX4_GZ_STANDALONE would be the documented
    # way, but in this PX4 version it never sees the world and times out.)
    gz_env = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs', 'gz_env.sh')
    if not os.path.isfile(gz_env):
        raise RuntimeError(f'{gz_env} not found: build PX4 first (make px4_sitl)')
    actions = [ExecuteProcess(
        cmd=['bash', '-c', f'. "{gz_env}" && exec gz sim --verbose=1 -r -s "{world_sdf}"'],
        output='screen')]
    if not headless:
        actions.append(ExecuteProcess(
            cmd=['bash', '-c', f'. "{gz_env}" && exec gz sim -g'], output='log'))
    env.update(PX4_GZ_MODELS=os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models'))
    world = cfg['world']
    wait_then_make = (
        'for i in $(seq 1 120); do '
        f'  gz service -i --service /world/{world}/scene/info 2>/dev/null '
        '    | grep -q "Service providers" && break; sleep 1; '
        'done; '
        f'if [ "$i" = 120 ]; then echo "{TAG} ERROR: Gazebo world {world} did not start"; '
        'exit 1; fi; '
        f'echo "{TAG} Gazebo world {world} is up after ${{i}} s, starting PX4"; '
        'exec ' + ' '.join(make))
    actions.append(ExecuteProcess(cmd=['bash', '-c', wait_then_make], cwd=px4_dir,
                                  additional_env=env, output='screen'))
    return actions


def _focus_camera_on_drone(context):
    """Point the Gazebo GUI camera at the drone once, then leave it free.

    The GUI opens with the world's default view, usually far from the spawn
    point. This calls the GUI's /gui/move_to/pose service (retrying until the
    GUI is up) with a camera behind and above the drone, looking at the point
    it will hover at. Skipped when headless (no GUI) or follow:=true (PX4
    already locks the camera on).
    """
    if _flag(context, 'headless') or _flag(context, 'follow'):
        return []
    cfg = context.launch_configurations
    pose = (cfg['pose'] or '0,0,0').split(',')
    x, y, z = (float(v) for v in (pose + ['0', '0', '0'])[:3])
    dx, dy, dz = (float(v) for v in cfg['camera_offset'].split(','))
    hover = float(cfg['altitude'])
    to_x, to_y, to_z = -dx, -dy, hover - dz
    yaw = math.atan2(to_y, to_x)
    pitch = math.atan2(-to_z, math.hypot(to_x, to_y))  # positive = looking down
    qw = math.cos(yaw / 2) * math.cos(pitch / 2)
    qx = -math.sin(yaw / 2) * math.sin(pitch / 2)
    qy = math.cos(yaw / 2) * math.sin(pitch / 2)
    qz = math.sin(yaw / 2) * math.cos(pitch / 2)
    req = (f'pose: {{position: {{x: {x + dx:.3f}, y: {y + dy:.3f}, z: {z + dz:.3f}}}, '
           f'orientation: {{x: {qx:.5f}, y: {qy:.5f}, z: {qz:.5f}, w: {qw:.5f}}}}}')
    call = ('gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera '
            f'--reptype gz.msgs.Boolean --timeout 2000 --req "{req}"')
    # Wait for the GUI (up to ~3 min: the first start can be slow), then call
    # twice a couple of seconds apart in case the GUI resets its view on load.
    script = (
        'for i in $(seq 1 180); do '
        '  gz service -l 2>/dev/null | grep -qx /gui/move_to/pose && break; sleep 1; '
        'done; '
        f'sleep 3; {call} >/dev/null; sleep 3; {call} >/dev/null && '
        f'echo "{TAG} Gazebo camera pointed at the drone (free to move)"'
    )
    return [ExecuteProcess(cmd=['bash', '-c', script], output='screen')]


def generate_launch_description():
    nav_share = get_package_share_directory('x500_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    args = [
        DeclareLaunchArgument(
            'world', default_value='baylands',
            description='Gazebo world name (the <world name> in the SDF).'),
        DeclareLaunchArgument(
            'world_sdf', default_value='',
            description='Path to a world SDF. Empty: use the PX4 world called "world".'),
        DeclareLaunchArgument(
            'px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot'),
            description='PX4-Autopilot source folder (built with make px4_sitl).'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true: run Gazebo without its window.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(nav_share, 'rviz', 'sim_baylands.rviz'),
            description='RViz configuration file.'),
        DeclareLaunchArgument(
            'follow', default_value='false',
            description='true: Gazebo camera follows the drone (locks the view). '
                        'false: free camera you can rotate, pan and zoom.'),
        DeclareLaunchArgument(
            'camera_offset', default_value='-2.5,-2.5,2.5',
            description='Start view of the free camera: offset "x,y,z" in m from the '
                        'spawn point. It looks at the drone and can then be moved freely.'),
        DeclareLaunchArgument(
            'follow_offset', default_value='-4.0,-4.0,3.0',
            description='With follow:=true, camera offset "x,y,z" from the drone in m.'),
        DeclareLaunchArgument(
            'pose', default_value='0,0,0,0,0,0',
            description='Spawn pose "x,y,z,roll,pitch,yaw" in Gazebo world metres.'),
        DeclareLaunchArgument(
            'altitude', default_value='2.0',
            description='Flying height in metres. The lidar scans at this height.'),
        DeclareLaunchArgument(
            'max_speed', default_value='1.0',
            description='Horizontal speed limit in m/s (applied by x500_px4_bridge).'),
        DeclareLaunchArgument(
            'safety', default_value='true',
            description='Lidar emergency stop in x500_px4_bridge.'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(nav_share, 'config', 'nav2_baylands.yaml'),
            description='Nav2 parameter file.'),
        DeclareLaunchArgument(
            'explore', default_value='false',
            description='true: explore the world on its own with explore_lite.'),
        DeclareLaunchArgument(
            'explore_params',
            default_value=os.path.join(nav_share, 'config', 'explore_office.yaml'),
            description='explore_lite parameter file.'),
        DeclareLaunchArgument(
            'explore_delay', default_value='8.0',
            description='Seconds between take-off and the start of exploring (climb time).'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(nav_share, 'config', 'slam_baylands.yaml'),
            description='slam_toolbox parameter file.'),
    ]

    # 2. PX4 <-> ROS 2.
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen')

    # 3. Gazebo -> ROS 2 (/clock, /scan). Not ros_gz_bridge: the Humble apt
    #    build talks to Gazebo Fortress, not Harmonic. See gz_bridge.py.
    #    Respawned if it ever exits: without /clock every sim-time node
    #    freezes and PX4 loses its OFFBOARD setpoints.
    gz_bridge = Node(
        package='x500_px4_bridge',
        executable='gz_bridge',
        parameters=[{'world': LaunchConfiguration('world'), 'model': f'{MODEL}_0'}],
        respawn=True,
        respawn_delay=1.0,
        output='screen',
    )

    # 4. TF tree: map -(slam)-> odom -(PX4: x, y, yaw)-> base_footprint
    #    -(PX4: height, roll, pitch)-> base_link -(fixed)-> link (lidar).
    #    map, odom and base_footprint are all at floor level.
    odom_tf = Node(
        package='x500_px4_bridge',
        executable='tf2_broadcaster',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    # Lidar mount from x500_lidar_2d/model.sdf (0.12, 0, 0.26) plus the
    # sensor's own 0.055 m offset in lidar_2d_v2/model.sdf.
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.12', '--y', '0', '--z', '0.315',
                   '--frame-id', 'base_link', '--child-frame-id', 'link'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 5. SLAM.
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[LaunchConfiguration('slam_params'), {'use_sim_time': True}],
        output='screen',
    )

    # 6. Nav2, with our drone behaviour tree (no Spin/BackUp recoveries; see
    #    behavior_trees/navigate_drone.xml) put into the params at its
    #    installed path.
    nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params'),
        param_rewrites={'default_nav_to_pose_bt_xml': os.path.join(
            nav_share, 'behavior_trees', 'navigate_drone.xml')},
        convert_types=True)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'autostart': 'true',
        }.items(),
    )

    # 7. Nav2 -> PX4. Retries OFFBOARD/ARM until PX4 has booted.
    px4_bridge = Node(
        package='x500_px4_bridge',
        executable='x500_px4_bridge',
        parameters=[{
            'use_sim_time': True,
            'target_altitude_m': PythonExpression(
                ['float(', LaunchConfiguration('altitude'), ')']),
            'max_xy_speed_mps': PythonExpression(
                ['float(', LaunchConfiguration('max_speed'), ')']),
            'safety_enabled': LaunchConfiguration('safety'),
        }],
        output='screen',
    )

    # 8. RViz.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    # 9. Autonomous exploration. Started from the bridge's own "taking off"
    #    log line, so it never sends goals while the drone is on the ground.
    explore = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=[LaunchConfiguration('explore_params'), {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('explore')),
        output='screen',
    )
    started = []

    def _on_bridge_output(event):
        if started or b'taking off' not in event.text:
            return None
        started.append(True)
        return [TimerAction(period=LaunchConfiguration('explore_delay'), actions=[explore])]

    start_explore = RegisterEventHandler(OnProcessIO(
        target_action=px4_bridge, on_stdout=_on_bridge_output, on_stderr=_on_bridge_output))

    # Ctrl+C only reaches `make`; the px4 binary and Gazebo keep running.
    # Kill them here (launch skips ExecuteProcess during shutdown, so this
    # calls pkill directly).
    stop_sim = RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=_stop_px4_and_gazebo),
    ]))

    actions = [
        OpaqueFunction(function=_focus_camera_on_drone),
        agent,
        OpaqueFunction(function=_simulator),
        gz_bridge,
        odom_tf,
        lidar_tf,
        stop_sim,
        start_explore,
        # Give PX4 and Gazebo time to start so SLAM and Nav2 find TF and /scan.
        TimerAction(period=15.0, actions=[slam, nav2, px4_bridge, rviz]),
    ]
    # Everything starts only after the preflight check (see _preflight).
    return LaunchDescription([
        *args,
        OpaqueFunction(function=_preflight, args=[actions]),
    ])
