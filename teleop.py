import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class KeyboardDrone(Node):

    def __init__(self):
        super().__init__('keyboard_drone')

        self.settings = termios.tcgetattr(sys.stdin)

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vyaw = 0.0

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.get_logger().info("Keyboard drone controller started")
        self.get_logger().info(
            "W/S: forward/backward | A/D: left/right"
        )
        self.get_logger().info(
            "R/F: up/down | Q/E: yaw left/right"
        )
        self.get_logger().info(
            "SPACE: stop | O: Offboard | T: Arm | G: Disarm | X: Exit"
        )

    def timer_callback(self):

        self.keyboard_callback()

        timestamp = self.get_clock().now().nanoseconds // 1000

        msg = OffboardControlMode()

        msg.timestamp = timestamp

        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

        trajectory = TrajectorySetpoint()

        trajectory.timestamp = timestamp

        trajectory.velocity = [
            self.vx,
            self.vy,
            self.vz
        ]

        trajectory.yawspeed = self.vyaw

        self.trajectory_pub.publish(trajectory)

    def publish_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        msg.param1 = param1
        msg.param2 = param2

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    def arm(self):

        self.get_logger().info("Arming drone")

        self.publish_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

    def disarm(self):

        self.get_logger().info("Disarming drone")

        self.publish_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0
        )

    def set_offboard_mode(self):

        self.get_logger().info("Requesting Offboard mode")

        self.publish_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

    def get_key(self):

        tty.setcbreak(sys.stdin.fileno())

        key = None

        if select.select(
            [sys.stdin],
            [],
            [],
            0.0
        )[0]:

            key = sys.stdin.read(1)

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.settings
        )

        return key

    def keyboard_callback(self):

        key = self.get_key()

        speed = 1.0
        yaw_speed = 0.5

        if key == 'w':

            self.vx = speed
            self.get_logger().info("Forward")

        elif key == 's':

            self.vx = -speed
            self.get_logger().info("Backward")

        elif key == 'a':

            self.vy = -speed
            self.get_logger().info("Left")

        elif key == 'd':

            self.vy = speed
            self.get_logger().info("Right")

        elif key == 'r':

            self.vz = -speed
            self.get_logger().info("Up")

        elif key == 'f':

            self.vz = speed
            self.get_logger().info("Down")

        elif key == 'q':

            self.vyaw = -yaw_speed
            self.get_logger().info("Yaw left")

        elif key == 'e':

            self.vyaw = yaw_speed
            self.get_logger().info("Yaw right")

        elif key == ' ':

            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
            self.vyaw = 0.0

            self.get_logger().info("STOP")

        elif key == 'o':

            self.set_offboard_mode()

        elif key == 't':

            self.arm()

        elif key == 'g':

            self.disarm()

        elif key == 'x':

            self.get_logger().info("Exiting controller")

            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
            self.vyaw = 0.0

            rclpy.shutdown()

    def cleanup(self):

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.settings
        )


def main(args=None):

    rclpy.init(args=args)

    drone = KeyboardDrone()

    try:

        rclpy.spin(drone)

    except KeyboardInterrupt:

        pass

    finally:

        drone.cleanup()

        drone.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()
