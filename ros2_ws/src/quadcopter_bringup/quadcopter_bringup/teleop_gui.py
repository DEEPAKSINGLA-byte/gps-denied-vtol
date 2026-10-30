#!/usr/bin/env python3
import sys
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand


class TeleopGUI(Node):
    def __init__(self, root):
        super().__init__('px4_teleop_gui')
        self.root = root
        self.root.title("PX4 Drone Teleop GUI")
        self.root.geometry("400x550")
        self.root.resizable(False, False)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.speed = 0.5
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.offboard_setpoint_counter = 0
        self.armed = False
        self.offboard_mode = False

        self.setup_ui()
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        title_label = ttk.Label(main_frame, text="PX4 Drone Teleop", font=("Arial", 16, "bold"))
        title_label.pack(pady=(0, 10))

        sep = ttk.Separator(main_frame, orient='horizontal')
        sep.pack(fill=tk.X, pady=5)

        speed_frame = ttk.LabelFrame(main_frame, text="Speed Control", padding="10")
        speed_frame.pack(fill=tk.X, pady=5)

        self.speed_var = tk.DoubleVar(value=self.speed)
        speed_scale = ttk.Scale(speed_frame, from_=0.1, to=5.0, variable=self.speed_var,
                                orient=tk.HORIZONTAL, command=self.on_speed_change)
        speed_scale.pack(fill=tk.X, pady=5)

        self.speed_label = ttk.Label(speed_frame, text=f"Speed: {self.speed:.2f} m/s")
        self.speed_label.pack()

        sep2 = ttk.Separator(main_frame, orient='horizontal')
        sep2.pack(fill=tk.X, pady=5)

        control_frame = ttk.LabelFrame(main_frame, text="Movement Control", padding="10")
        control_frame.pack(fill=tk.X, pady=5)

        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack()

        btn_style = {"width": 8, "padding": 5}

        ttk.Button(btn_frame, text="↑ Forward (W)", command=lambda: self.set_velocity(1.0, 0.0, 0.0), **btn_style).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="← Left (A)", command=lambda: self.set_velocity(0.0, 1.0, 0.0), **btn_style).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(btn_frame, text="Stop (X)", command=lambda: self.set_velocity(0.0, 0.0, 0.0), **btn_style).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="Right (D) →", command=lambda: self.set_velocity(0.0, -1.0, 0.0), **btn_style).grid(row=1, column=2, padx=2, pady=2)
        ttk.Button(btn_frame, text="↓ Back (S)", command=lambda: self.set_velocity(-1.0, 0.0, 0.0), **btn_style).grid(row=2, column=1, padx=2, pady=2)

        alt_frame = ttk.Frame(control_frame)
        alt_frame.pack(pady=10)
        ttk.Button(alt_frame, text="↑ Up (R)", command=lambda: self.set_velocity(0.0, 0.0, -1.0), **btn_style).pack(side=tk.LEFT, padx=5)
        ttk.Button(alt_frame, text="↓ Down (F)", command=lambda: self.set_velocity(0.0, 0.0, 1.0), **btn_style).pack(side=tk.LEFT, padx=5)

        sep3 = ttk.Separator(main_frame, orient='horizontal')
        sep3.pack(fill=tk.X, pady=5)

        action_frame = ttk.LabelFrame(main_frame, text="Vehicle Commands", padding="10")
        action_frame.pack(fill=tk.X, pady=5)

        self.offboard_btn = ttk.Button(action_frame, text="Enable OFFBOARD", command=self.toggle_offboard)
        self.offboard_btn.pack(fill=tk.X, pady=2)

        self.arm_btn = ttk.Button(action_frame, text="ARM", command=self.toggle_arm)
        self.arm_btn.pack(fill=tk.X, pady=2)

        ttk.Button(action_frame, text="Disarm & Land", command=self.disarm_land).pack(fill=tk.X, pady=2)

        sep4 = ttk.Separator(main_frame, orient='horizontal')
        sep4.pack(fill=tk.X, pady=5)

        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="10")
        status_frame.pack(fill=tk.X, pady=5)

        self.status_labels = {}
        for label_text, var_name in [("Velocity X:", "vx"), ("Velocity Y:", "vy"), ("Velocity Z:", "vz"),
                                      ("Speed:", "speed"), ("OFFBOARD:", "offboard"), ("ARMED:", "armed")]:
            frame = ttk.Frame(status_frame)
            frame.pack(fill=tk.X, pady=1)
            ttk.Label(frame, text=label_text, width=12).pack(side=tk.LEFT)
            self.status_labels[var_name] = ttk.Label(frame, text="0.00", font=("Monospace", 10))
            self.status_labels[var_name].pack(side=tk.LEFT)

        self.update_status()

    def on_speed_change(self, val):
        self.speed = float(val)
        self.speed_label.config(text=f"Speed: {self.speed:.2f} m/s")
        self.update_status()

    def set_velocity(self, vx, vy, vz):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.update_status()

    def toggle_offboard(self):
        if not self.offboard_mode:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Requested OFFBOARD mode")
            self.offboard_mode = True
            self.offboard_btn.config(text="Disable OFFBOARD")
        else:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0)
            self.get_logger().info("Disabled OFFBOARD mode")
            self.offboard_mode = False
            self.offboard_btn.config(text="Enable OFFBOARD")
        self.update_status()

    def toggle_arm(self):
        if not self.armed:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Sent ARM command")
            self.armed = True
            self.arm_btn.config(text="DISARM")
        else:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            self.get_logger().info("Sent DISARM command")
            self.armed = False
            self.arm_btn.config(text="ARM")
        self.update_status()

    def disarm_land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarmed and landed")
        self.armed = False
        self.offboard_mode = False
        self.offboard_btn.config(text="Enable OFFBOARD")
        self.arm_btn.config(text="ARM")
        self.set_velocity(0.0, 0.0, 0.0)
        self.update_status()

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
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter == 15 and not self.offboard_mode:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Auto-requested OFFBOARD mode")
            self.offboard_mode = True
            self.offboard_btn.config(text="Disable OFFBOARD")
        elif self.offboard_setpoint_counter == 30 and not self.armed:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Auto-sent ARM command")
            self.armed = True
            self.arm_btn.config(text="DISARM")

        if self.offboard_setpoint_counter < 100:
            self.offboard_setpoint_counter += 1

        self.update_status()

    def update_status(self):
        self.status_labels["vx"].config(text=f"{self.vx * self.speed:.2f}")
        self.status_labels["vy"].config(text=f"{self.vy * self.speed:.2f}")
        self.status_labels["vz"].config(text=f"{self.vz * self.speed:.2f}")
        self.status_labels["speed"].config(text=f"{self.speed:.2f}")
        self.status_labels["offboard"].config(text="YES" if self.offboard_mode else "NO")
        self.status_labels["armed"].config(text="YES" if self.armed else "NO")

    def on_closing(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


def main():
    rclpy.init()
    root = tk.Tk()
    gui = TeleopGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()