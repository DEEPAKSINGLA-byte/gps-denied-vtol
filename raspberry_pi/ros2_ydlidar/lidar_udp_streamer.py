#!/usr/bin/env python3
"""
Lightweight LiDAR UDP Streamer for Raspberry Pi (Companion Computer)
Streams live YDLIDAR G2 scan packets over Wi-Fi / Ethernet UDP to Ground Station.

Wire format (default --format bin, ~1.1 KB -> one UDP datagram, never IP-fragmented):
    <4s magic b'YDL1'> <f64 capture time> <u16 n>  <n x u16 range mm> <n x u8 intensity>
The old JSON format (~4 KB, split into 3 IP fragments; losing any one drops the scan
on Wi-Fi) is still available with --format json. The receivers accept both.
"""
import time
import socket
import struct
import json
import argparse
import serial
import sys

LIDAR_MAGIC = b'YDL1'
_HDR = struct.Struct('<4sdH')


def encode_scan(ts, ranges, intensities, fmt):
    if fmt == 'json':
        return json.dumps({"timestamp": ts, "ranges": [round(r, 3) for r in ranges],
                           "intensities": intensities}).encode()
    n = len(ranges)
    mm = [min(int(r * 1000.0 + 0.5), 65535) for r in ranges]
    return (_HDR.pack(LIDAR_MAGIC, ts, n) + struct.pack(f'<{n}H', *mm)
            + bytes(min(i, 255) for i in intensities))

def main():
    parser = argparse.ArgumentParser(description="YDLIDAR G2 UDP Streamer")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (115200 for G2B)")
    parser.add_argument("--target-ip", default="192.168.137.1", help="Target ground station / laptop IP")
    parser.add_argument("--udp-port", type=int, default=14552, help="UDP transmission port")
    parser.add_argument("--format", choices=["bin", "json"], default="bin",
                        help="Packet format (default: bin, compact single-datagram)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # DSCP CS6 -> Wi-Fi WMM voice queue: scans skip ahead of bulk camera traffic.
    try:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xC0)
    except OSError:
        pass
    print(f"[STREAMER] Streaming LiDAR data from {args.port} -> {args.target_ip}:{args.udp_port}")

    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    try:  # USB-serial adapters (FTDI) otherwise batch bytes for up to 16 ms
        import fcntl, array
        TIOCGSERIAL, TIOCSSERIAL, ASYNC_LOW_LATENCY = 0x541E, 0x541F, 1 << 13
        buf = array.array('i', [0] * 32)
        fcntl.ioctl(ser.fileno(), TIOCGSERIAL, buf)
        buf[4] |= ASYNC_LOW_LATENCY
        fcntl.ioctl(ser.fileno(), TIOCSSERIAL, buf)
    except Exception:
        pass
    ser.dtr = False
    ser.rts = True
    time.sleep(0.3)
    # Start scan
    ser.write(bytes([0xA5, 0x60]))
    header = ser.read(7)
    print(f"[STREAMER] Handshake header: {header.hex()}")

    ranges = [0.0] * 360
    intensities = [0] * 360
    rev_start = time.time()
    last_start_deg = None

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

            lsn = hdr[1]
            fsa = hdr[2] | (hdr[3] << 8)
            lsa = hdr[4] | (hdr[5] << 8)

            pdata = ser.read(lsn * 3)
            if len(pdata) < lsn * 3:
                continue

            start_deg = (fsa >> 1) / 64.0
            end_deg = (lsa >> 1) / 64.0
            diff = (end_deg - start_deg) % 360.0

            # New revolution = the packet start angle wraps past 360 deg. Do NOT rely on the
            # CT "start packet" flag: G2 firmware 3.5 sets it only every ~3-4 turns (measured:
            # 6.4 rev/s, 2 flags/s), which merged several turns into one scan (2 Hz, ~200 ms old).
            wrapped = last_start_deg is not None and start_deg + 180.0 < last_start_deg
            last_start_deg = start_deg
            if wrapped:
                # 1 revolution finished, transmit UDP packet. Stamp with the middle of the
                # revolution (when the data was actually measured), not the send time.
                now = time.time()
                ts = 0.5 * (rev_start + now)
                rev_start = now
                try:
                    sock.sendto(encode_scan(ts, ranges, intensities, args.format),
                                (args.target_ip, args.udp_port))
                except OSError:
                    pass  # network down / no route: drop this scan, never block the serial loop
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
        except serial.SerialException as e:
            # Unplugged USB: exit so the supervisor (start_streams.py) restarts us,
            # instead of spinning forever in this loop at 100% CPU.
            print(f"[STREAMER] Serial error ({e}); exiting")
            sys.exit(1)
        except Exception:
            continue

    ser.write(bytes([0xA5, 0x65]))
    ser.close()
    sock.close()

if __name__ == "__main__":
    main()
