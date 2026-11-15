#!/usr/bin/env python3
"""
plotjuggler_bridge.py - Real-Time Drone Telemetry Bridge for PlotJuggler
=======================================================================
Receives live Pixhawk flight telemetry over the network from Raspberry Pi (Redmi 10)
and forwards structured timeseries streams directly to PlotJuggler via:

  1. UDP Server (JSON): Streams formatted metrics to 127.0.0.1:9870
  2. ROS 2 Topics (Optional): Publishes standard sensor_msgs/Imu, Vector3Stamped, etc.
  3. Live CSV Logger: Continuously updates plotjuggler_live.csv for offline/drag-and-drop analysis.

Usage Examples:
  # Start bridge (forwards to PlotJuggler UDP port 9870)
  python3 plotjuggler_bridge.py

  # Start bridge with ROS 2 topic publishing
  python3 plotjuggler_bridge.py --ros2

  # Custom input / output ports
  python3 plotjuggler_bridge.py --listen-port 14551 --pj-port 9870
"""

import sys
import os
import time
import json
import socket
import argparse
from datetime import datetime

# Optional ROS 2 integration
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, BatteryState
    from geometry_msgs.msg import Vector3Stamped
    from std_msgs.msg import Float32
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def flatten_telemetry(telem: dict) -> dict:
    """
    Flatten nested telemetry into clean, PlotJuggler-friendly hierarchical keys.
    """
    flat = {
        "timestamp": telem.get("timestamp", time.time()),
        "status/armed": 1 if telem.get("armed") else 0,
    }

    # Attitude
    att = telem.get("attitude") or {}
    if att:
        flat["attitude/roll_deg"] = att.get("roll_deg", 0.0)
        flat["attitude/pitch_deg"] = att.get("pitch_deg", 0.0)
        flat["attitude/yaw_deg"] = att.get("yaw_deg", 0.0)
        flat["attitude/rollspeed_dps"] = att.get("rollspeed_dps", 0.0)
        flat["attitude/pitchspeed_dps"] = att.get("pitchspeed_dps", 0.0)
        flat["attitude/yawspeed_dps"] = att.get("yawspeed_dps", 0.0)

    # IMU
    imu = telem.get("imu") or {}
    if imu:
        flat["imu/accel_x_mg"] = imu.get("accel_x_mg", 0.0)
        flat["imu/accel_y_mg"] = imu.get("accel_y_mg", 0.0)
        flat["imu/accel_z_mg"] = imu.get("accel_z_mg", 0.0)
        flat["imu/gyro_x_dps"] = imu.get("gyro_x_dps", 0.0)
        flat["imu/gyro_y_dps"] = imu.get("gyro_y_dps", 0.0)
        flat["imu/gyro_z_dps"] = imu.get("gyro_z_dps", 0.0)
        flat["imu/mag_x_mgauss"] = imu.get("mag_x_mgauss", 0.0)
        flat["imu/mag_y_mgauss"] = imu.get("mag_y_mgauss", 0.0)
        flat["imu/mag_z_mgauss"] = imu.get("mag_z_mgauss", 0.0)

    # Barometer & Temp
    baro = telem.get("barometer") or {}
    if baro:
        flat["baro/pressure_hpa"] = baro.get("pressure_hpa", 0.0)
        flat["baro/temperature_c"] = baro.get("temperature_c", 0.0)

    # Battery
    bat = telem.get("battery") or {}
    if bat:
        flat["battery/voltage_v"] = bat.get("voltage_v", 0.0)
        flat["battery/current_a"] = bat.get("current_a") or 0.0
        flat["battery/remaining_pct"] = bat.get("remaining_pct") or 0.0

    # GPS
    gps = telem.get("gps") or {}
    if gps:
        flat["gps/satellites"] = gps.get("satellites_visible", 0)
        flat["gps/fix_type"] = gps.get("fix_type", 0)

    # HUD & Navigation
    hud = telem.get("vfr_hud") or {}
    if hud:
        flat["hud/alt_m"] = hud.get("alt_m", 0.0)
        flat["hud/heading_deg"] = hud.get("heading_deg", 0)
        flat["hud/airspeed_ms"] = hud.get("airspeed_ms", 0.0)
        flat["hud/groundspeed_ms"] = hud.get("groundspeed_ms", 0.0)
        flat["hud/throttle_pct"] = hud.get("throttle_pct", 0)
        flat["hud/climb_ms"] = hud.get("climb_ms", 0.0)

    # Local NED (if available)
    lpos = telem.get("local_position_ned") or {}
    if lpos:
        flat["local_ned/x_m"] = lpos.get("x_m", 0.0)
        flat["local_ned/y_m"] = lpos.get("y_m", 0.0)
        flat["local_ned/z_m"] = lpos.get("z_m", 0.0)
        flat["local_ned/vx_ms"] = lpos.get("vx_ms", 0.0)
        flat["local_ned/vy_ms"] = lpos.get("vy_ms", 0.0)
        flat["local_ned/vz_ms"] = lpos.get("vz_ms", 0.0)

    # Rangefinder (if available)
    rf = telem.get("rangefinder") or {}
    if rf:
        flat["rangefinder/distance_m"] = rf.get("distance_m", 0.0)

    return flat


class ROS2DronePublisher(Node):
    def __init__(self):
        super().__init__("drone_telemetry_publisher")
        self.pub_imu = self.create_publisher(Imu, "/drone/imu", 10)
        self.pub_att = self.create_publisher(Vector3Stamped, "/drone/attitude", 10)
        self.pub_bat = self.create_publisher(BatteryState, "/drone/battery", 10)
        self.pub_alt = self.create_publisher(Float32, "/drone/altitude", 10)

    def publish_telemetry(self, flat: dict):
        now = self.get_clock().now().to_msg()

        # 1. Attitude (Roll, Pitch, Yaw in degrees)
        msg_att = Vector3Stamped()
        msg_att.header.stamp = now
        msg_att.header.frame_id = "base_link"
        msg_att.vector.x = float(flat.get("attitude/roll_deg", 0.0))
        msg_att.vector.y = float(flat.get("attitude/pitch_deg", 0.0))
        msg_att.vector.z = float(flat.get("attitude/yaw_deg", 0.0))
        self.pub_att.publish(msg_att)

        # 2. IMU
        msg_imu = Imu()
        msg_imu.header.stamp = now
        msg_imu.header.frame_id = "imu_link"
        # Convert mg to m/s^2 (1 mg = 0.00980665 m/s^2)
        msg_imu.linear_acceleration.x = float(flat.get("imu/accel_x_mg", 0.0)) * 0.00980665
        msg_imu.linear_acceleration.y = float(flat.get("imu/accel_y_mg", 0.0)) * 0.00980665
        msg_imu.linear_acceleration.z = float(flat.get("imu/accel_z_mg", 0.0)) * 0.00980665
        # Convert deg/s to rad/s
        msg_imu.angular_velocity.x = float(flat.get("imu/gyro_x_dps", 0.0)) * 0.0174533
        msg_imu.angular_velocity.y = float(flat.get("imu/gyro_y_dps", 0.0)) * 0.0174533
        msg_imu.angular_velocity.z = float(flat.get("imu/gyro_z_dps", 0.0)) * 0.0174533
        self.pub_imu.publish(msg_imu)

        # 3. Altitude
        msg_alt = Float32()
        msg_alt.data = float(flat.get("hud/alt_m", 0.0))
        self.pub_alt.publish(msg_alt)

        # 4. Battery
        msg_bat = BatteryState()
        msg_bat.header.stamp = now
        msg_bat.voltage = float(flat.get("battery/voltage_v", 0.0))
        msg_bat.current = float(flat.get("battery/current_a", 0.0))
        msg_bat.percentage = float(flat.get("battery/remaining_pct", 0.0)) / 100.0
        self.pub_bat.publish(msg_bat)


def run_bridge(
    listen_port: int = 14551,
    pj_port: int = 9870,
    enable_ros2: bool = False,
    csv_log_path: str = "plotjuggler_live.csv"
):
    print("=" * 70)
    print(" PLOTJUGGLER DRONE TELEMETRY BRIDGE")
    print("=" * 70)
    print(f"Listening on:            0.0.0.0:{listen_port} (UDP from RPi)")
    print(f"Forwarding to PJ UDP:    127.0.0.1:{pj_port} (JSON format)")
    print(f"ROS 2 Publishing:        {'ENABLED (/drone/*)' if enable_ros2 and ROS2_AVAILABLE else 'DISABLED'}")
    print(f"Live CSV Log:            {csv_log_path}")
    print("=" * 70)

    # Socket to receive from Raspberry Pi
    in_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    in_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    in_sock.bind(("0.0.0.0", listen_port))

    # Socket to forward to PlotJuggler UDP Server
    pj_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pj_target = ("127.0.0.1", pj_port)

    # ROS 2 setup
    ros_node = None
    if enable_ros2:
        if ROS2_AVAILABLE:
            rclpy.init(args=None)
            ros_node = ROS2DronePublisher()
            print("[+] ROS 2 topics initialized: /drone/attitude, /drone/imu, /drone/altitude, /drone/battery")
        else:
            print("[WARN] ROS 2 packages not found. Skipping ROS 2 topic publishing.")

    # CSV File setup
    csv_file = open(csv_log_path, "w", buffering=1)
    csv_header_written = False

    packet_count = 0
    start_time = time.time()

    print("\nWaiting for telemetry stream from Raspberry Pi...")
    try:
        while True:
            data, addr = in_sock.recvfrom(65535)
            try:
                raw_telem = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            packet_count += 1
            flat_telem = flatten_telemetry(raw_telem)

            # 1. Forward to PlotJuggler UDP Server (JSON format)
            pj_payload = json.dumps(flat_telem).encode("utf-8")
            pj_sock.sendto(pj_payload, pj_target)

            # 2. Publish to ROS 2 (if enabled)
            if ros_node:
                ros_node.publish_telemetry(flat_telem)
                rclpy.spin_once(ros_node, timeout_sec=0.001)

            # 3. Write to live CSV
            if not csv_header_written:
                header = ",".join(flat_telem.keys()) + "\n"
                csv_file.write(header)
                csv_header_written = True
            row = ",".join(str(v) for v in flat_telem.values()) + "\n"
            csv_file.write(row)

            # Status printout
            if packet_count % 10 == 0:
                elapsed = time.time() - start_time
                fps = packet_count / elapsed if elapsed > 0 else 0
                roll = flat_telem.get("attitude/roll_deg", 0.0)
                pitch = flat_telem.get("attitude/pitch_deg", 0.0)
                yaw = flat_telem.get("attitude/yaw_deg", 0.0)
                az = flat_telem.get("imu/accel_z_mg", 0.0)
                alt = flat_telem.get("hud/alt_m", 0.0)

                sys.stdout.write(
                    f"\r[PLOTJUGGLER BRIDGE] Pkts: {packet_count:>5} ({fps:>4.1f} Hz) | "
                    f"Roll: {roll:>5.1f}° Pitch: {pitch:>5.1f}° Yaw: {yaw:>6.1f}° | "
                    f"AccelZ: {az:>6.0f}mg | Alt: {alt:>4.1f}m -> Forwarded to PJ UDP {pj_port}"
                )
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nStopping PlotJuggler bridge...")
    finally:
        in_sock.close()
        pj_sock.close()
        csv_file.close()
        if ros_node:
            ros_node.destroy_node()
            rclpy.shutdown()
        print("Bridge stopped.")


def main():
    parser = argparse.ArgumentParser(description="PlotJuggler Telemetry Bridge")
    parser.add_argument("--listen-port", type=int, default=14551, help="UDP port receiving data from RPi (default: 14551)")
    parser.add_argument("--pj-port", type=int, default=9870, help="PlotJuggler UDP Server port (default: 9870)")
    parser.add_argument("--ros2", action="store_true", help="Publish telemetry to ROS 2 topics (/drone/*)")
    parser.add_argument("--csv", type=str, default="plotjuggler_live.csv", help="Live CSV output file (default: plotjuggler_live.csv)")

    args = parser.parse_args()
    run_bridge(
        listen_port=args.listen_port,
        pj_port=args.pj_port,
        enable_ros2=args.ros2,
        csv_log_path=args.csv
    )


if __name__ == "__main__":
    main()
