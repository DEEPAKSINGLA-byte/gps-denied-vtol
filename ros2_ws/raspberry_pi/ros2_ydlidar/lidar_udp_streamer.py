#!/usr/bin/env python3
"""
Lightweight LiDAR UDP Streamer for Raspberry Pi (Companion Computer)
Streams live YDLIDAR G2 scan packets over Wi-Fi / Ethernet UDP to Ground Station.
"""
import time
import socket
import json
import argparse
import serial

def main():
    parser = argparse.ArgumentParser(description="YDLIDAR G2 UDP Streamer")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (115200 for G2B)")
    parser.add_argument("--target-ip", default="192.168.137.1", help="Target ground station / laptop IP")
    parser.add_argument("--udp-port", type=int, default=14552, help="UDP transmission port")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[STREAMER] Streaming LiDAR data from {args.port} -> {args.target_ip}:{args.udp_port}")

    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    ser.dtr = False
    ser.rts = True
    time.sleep(0.3)
    # Start scan
    ser.write(bytes([0xA5, 0x60]))
    header = ser.read(7)
    print(f"[STREAMER] Handshake header: {header.hex()}")

    ranges = [0.0] * 360
    intensities = [0] * 360
    last_send = time.time()

    while True:
        try:
            b1 = ser.read(1)
            if not b1 or b1[0] != 0xAA:
                continue
            b2 = ser.read(1)
            if not b2 or b2[0] != 0x55:
                continue

            hdr = ser.read(8)
            if len(hdr) < 8:
                continue

            ct = hdr[0]
            lsn = hdr[1]
            fsa = hdr[2] | (hdr[3] << 8)
            lsa = hdr[4] | (hdr[5] << 8)

            pdata = ser.read(lsn * 3)
            if len(pdata) < lsn * 3:
                continue

            start_deg = (fsa >> 1) / 64.0
            end_deg = (lsa >> 1) / 64.0
            diff = (end_deg - start_deg) % 360.0

            if (ct & 0x01) == 0x01:
                # 1 revolution finished, transmit UDP packet
                now = time.time()
                payload = {
                    "timestamp": now,
                    "ranges": [round(r, 3) for r in ranges],
                    "intensities": intensities
                }
                sock.sendto(json.dumps(payload).encode(), (args.target_ip, args.udp_port))
                ranges = [0.0] * 360
                intensities = [0] * 360

            for i in range(lsn):
                sample = pdata[i * 3: (i + 1) * 3]
                inten = sample[0]
                dist_raw = sample[1] | (sample[2] << 8)
                dist_m = (dist_raw / 4.0) / 1000.0

                ang = (start_deg + (diff / (lsn - 1)) * i) % 360.0 if lsn > 1 else start_deg
                idx = int(ang) % 360
                if 0.12 <= dist_m <= 16.0:
                    ranges[idx] = dist_m
                    intensities[idx] = inten

        except KeyboardInterrupt:
            break
        except Exception as e:
            continue

    ser.write(bytes([0xA5, 0x65]))
    ser.close()
    sock.close()

if __name__ == "__main__":
    main()
