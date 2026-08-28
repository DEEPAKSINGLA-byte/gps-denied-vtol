#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand


# ============================================================
# USER INTERFACE
# ============================================================

msg = """
--------------------------------------------------
PX4 Drone Keyboard Teleop (uXRCE-DDS)
--------------------------------------------------

Movement:

             w : Forward  (+X)
      a      s      d
   Left    Back   Right
   (+Y)    (-X)   (-Y)

             r : Up      (-Z)
             f : Down    (+Z)

             x : STOP / HOVER

Speed:

        q : Increase speed by 10%
        z : Decrease speed by 10%

        CTRL+C : Exit

--------------------------------------------------
"""


# ============================================================
# KEY BINDINGS
# PX4 uses NED:
#
# +X = North / Forward
# +Y = East / Right depending on body/world frame
# +Z = Down
#
# ============================================================

moveBindings = {
    'w': (1.0, 0.0, 0.0),     # Forward
    's': (-1.0, 0.0, 0.0),    # Backward

    'a': (0.0, 1.0, 0.0),     # Left
    'd': (0.0, -1.0, 0.0),    # Right

    'r': (0.0, 0.0, -1.0),    # Up
    'f': (0.0, 0.0, 1.0),     # Down

    'x': (0.0, 0.0, 0.0),      # Stop
}


speedBindings = {
    'q': 1.1,
    'z': 0.9,
}


# ============================================================
# READ ALL AVAILABLE KEYS
# ============================================================

def get_keys():

    keys = []

    while True:

        rlist, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0
        )

        if not rlist:
            break

        key = sys.stdin.read(1)

        if key:
            keys.append(key)

    return keys


# ============================================================
# PX4 TELEOP NODE
# ============================================================

class PX4TeleopKey(Node):

    def __init__(self):

        super().__init__('px4_teleop_key')

        # ----------------------------------------------------
        # PX4 uXRCE-DDS QoS
        # ----------------------------------------------------

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----------------------------------------------------
        # PX4 Publishers
        # ----------------------------------------------------

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        # ----------------------------------------------------
        # Movement state
        # ----------------------------------------------------

        self.speed = 0.5

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # ----------------------------------------------------
        # PX4 Offboard initialization counter
        # ----------------------------------------------------

        self.offboard_setpoint_counter = 0

        # ----------------------------------------------------
        # 20 Hz control loop
        # ----------------------------------------------------

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.get_logger().info(
            "PX4 Keyboard Teleop started"
        )

    # ========================================================
    # TIMESTAMP
    # ========================================================

    def timestamp(self):

        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    # ========================================================
    # OFFBOARD CONTROL MODE
    # ========================================================

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        # We are controlling velocity
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher.publish(msg)

    # ========================================================
    # TRAJECTORY SETPOINT
    # ========================================================

    def publish_trajectory_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # ----------------------------------------------------
        # Position unused
        # ----------------------------------------------------

        msg.position = [
            float('nan'),
            float('nan'),
            float('nan')
        ]

        # ----------------------------------------------------
        # Velocity command
        # ----------------------------------------------------

        msg.velocity = [
            float(self.vx * self.speed),
            float(self.vy * self.speed),
            float(self.vz * self.speed)
        ]

        # ----------------------------------------------------
        # Acceleration unused
        # ----------------------------------------------------

        msg.acceleration = [
            float('nan'),
            float('nan'),
            float('nan')
        ]

        # ----------------------------------------------------
        # Yaw unused
        # ----------------------------------------------------

        msg.yaw = float('nan')
        msg.yawspeed = float('nan')

        self.trajectory_setpoint_publisher.publish(msg)

    # ========================================================
    # VEHICLE COMMAND
    # ========================================================

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = int(command)

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.vehicle_command_publisher.publish(msg)

    # ========================================================
    # ARM
    # ========================================================

    def arm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

        self.get_logger().info(
            "ARM command sent"
        )

    # ========================================================
    # SET OFFBOARD MODE
    # ========================================================

    def set_offboard_mode(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

        self.get_logger().info(
            "OFFBOARD mode command sent"
        )

    # ========================================================
    # PROCESS KEY
    # ========================================================

    def process_key(self, key):

        # ----------------------------------------------------
        # Movement
        # ----------------------------------------------------

        if key in moveBindings:

            self.vx = moveBindings[key][0]
            self.vy = moveBindings[key][1]
            self.vz = moveBindings[key][2]

            # Display command

            vx = self.vx * self.speed
            vy = self.vy * self.speed
            vz = self.vz * self.speed

            print(
                f"\rCommand: "
                f"VX={vx:+.2f} "
                f"VY={vy:+.2f} "
                f"VZ={vz:+.2f}       ",
                end="",
                flush=True
            )

        # ----------------------------------------------------
        # Speed
        # ----------------------------------------------------

        elif key in speedBindings:

            self.speed *= speedBindings[key]

            # Prevent absurdly low speed

            if self.speed < 0.05:
                self.speed = 0.05

            # Prevent absurdly high speed

            if self.speed > 10.0:
                self.speed = 10.0

            print(
                f"\rCurrent Speed: "
                f"{self.speed:.2f} m/s       ",
                end="",
                flush=True
            )

        # ----------------------------------------------------
        # CTRL+C
        # ----------------------------------------------------

        elif key == '\x03':

            self.get_logger().info(
                "CTRL+C received"
            )

            rclpy.shutdown()

    # ========================================================
    # MAIN TIMER CALLBACK
    # ========================================================

    def timer_callback(self):

        # ----------------------------------------------------
        # 1. Read ALL pending keyboard input
        # ----------------------------------------------------

        keys = get_keys()

        for key in keys:

            self.process_key(key)

        # ----------------------------------------------------
        # 2. PX4 Offboard heartbeat
        # ----------------------------------------------------

        self.publish_offboard_control_mode()

        # ----------------------------------------------------
        # 3. Publish velocity setpoint
        # ----------------------------------------------------

        self.publish_trajectory_setpoint()

        # ----------------------------------------------------
        # 4. Offboard initialization sequence
        # ----------------------------------------------------

        if self.offboard_setpoint_counter == 15:

            self.set_offboard_mode()

        elif self.offboard_setpoint_counter == 30:

            self.arm()

        # ----------------------------------------------------
        # Continue counting
        # ----------------------------------------------------

        if self.offboard_setpoint_counter < 100:

            self.offboard_setpoint_counter += 1


# ============================================================
# MAIN
# ============================================================

def main():

    # --------------------------------------------------------
    # Save original terminal settings
    # --------------------------------------------------------

    settings = termios.tcgetattr(sys.stdin)

    # --------------------------------------------------------
    # Put terminal into RAW mode ONCE
    #
    # Do NOT repeatedly call tty.setraw() inside the timer.
    # --------------------------------------------------------

    tty.setraw(sys.stdin.fileno())

    # --------------------------------------------------------
    # Initialize ROS2
    # --------------------------------------------------------

    rclpy.init()

    node = PX4TeleopKey()

    print(msg)

    try:

        # ----------------------------------------------------
        # ROS2 event loop
        # ----------------------------------------------------

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # ----------------------------------------------------
        # Restore terminal
        # ----------------------------------------------------

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings
        )

        # ----------------------------------------------------
        # Stop the drone before exiting
        # ----------------------------------------------------

        node.vx = 0.0
        node.vy = 0.0
        node.vz = 0.0

        # Publish zero velocity once
        # before shutting down.

        try:

            node.publish_offboard_control_mode()
            node.publish_trajectory_setpoint()

        except Exception:

            pass

        # ----------------------------------------------------
        # Destroy node
        # ----------------------------------------------------

        node.destroy_node()

        # ----------------------------------------------------
        # Shutdown ROS2
        # ----------------------------------------------------

        if rclpy.ok():

            rclpy.shutdown()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == '__main__':

    main()
