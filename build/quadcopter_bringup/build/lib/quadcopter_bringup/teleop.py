#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Native PX4 uORB message types
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus

msg = """
--------------------------------------------------
PX4 Drone Keyboard Teleop (uXRCE-DDS)
--------------------------------------------------
Moving around:
        w : Move Forward (+X)
   a    s    d : Move Left (+Y) / Move Back (-X) / Move Right (-Y)
        r : Move Up (-Z in NED frame)
        f : Move Down (+Z in NED frame)
        x : Stop / Hover in place

Q/Z : Increase/Decrease Max Speed by 10%
CTRL+C to exit safely.
--------------------------------------------------
"""

# Movement bindings: (vx, vy, vz)
moveBindings = {
    'w': ( 1.0,  0.0,  0.0),
    's': (-1.0,  0.0,  0.0),
    'a': ( 0.0,  1.0,  0.0),
    'd': ( 0.0, -1.0,  0.0),
    'r': ( 0.0,  0.0, -1.0), # In NED, negative Z goes UP
    'f': ( 0.0,  0.0,  1.0), # In NED, positive Z goes DOWN
    'x': ( 0.0,  0.0,  0.0),
}

speedBindings = {
    'q': 1.1,
    'z': 0.9,
}

def getKey(settings):
    # Non-blocking key check so the 20Hz loop doesn't freeze waiting for input
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class PX4TeleopKey(Node):
    def __init__(self, settings):
        super().__init__('px4_teleop_key')
        self.settings = settings

        # QoS profile required by PX4 uXRCE-DDS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PX4 Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Flight dynamics states
        self.speed = 0.5  # m/s
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.offboard_setpoint_counter = 0

        # Run control loop at 20 Hz (every 0.05 seconds)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Velocity control mode mapping
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float(self.vx * self.speed), float(self.vy * self.speed), float(self.vz * self.speed)]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')
        
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        # 1. Check for keyboard input non-blockingly
        key = getKey(self.settings)
        if key in moveBindings:
            self.vx = moveBindings[key][0]
            self.vy = moveBindings[key][1]
            self.vz = moveBindings[key][2]
        elif key in speedBindings:
            self.speed *= speedBindings[key]
            print(f"Current Speed Multiplier: {self.speed:.2f} m/s")
        elif key == '\x03':  # CTRL+C
            rclpy.shutdown()

        # 2. Continuous heartbeat requirements for PX4
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 3. Handle Handshake sequence (stream setpoints -> switch mode -> arm)
        if self.offboard_setpoint_counter == 15:
            # Request Offboard Mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Requested OFFBOARD mode")
        elif self.offboard_setpoint_counter == 30:
            # Send Arm Command
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Sent ARM command")

        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = PX4TeleopKey(settings)

    print(msg)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings on exit
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()