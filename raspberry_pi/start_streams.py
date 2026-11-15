#!/usr/bin/env python3
"""
start_streams.py - ONE command on the Raspberry Pi to stream everything to the laptop.

    python3 start_streams.py 10.184.3.231

Starts and supervises (auto-restart with back-off if one crashes):
    telemetry : telemetry_streamer.py   -> UDP 14551 JSON, 14550 MAVLink  (owns the Pixhawk link;
                                           also serves arm/mode commands on UDP 14552)
    lidar     : lidar_udp_streamer.py   -> UDP 14552
    camera    : camera_streamer.py      -> TCP 5000 (laptop must be listening; it reconnects)
    nav cmds  : optional (--nav-cmds), Nav2 velocity listener on UDP 14553, sharing the
                telemetry process's Pixhawk connection (no serial-port conflict).

Ctrl-C stops everything. Use --no-lidar / --no-camera / --no-telemetry to skip a stream.
If the target IP is omitted, the SSH client IP is used when run over ssh.
"""
import argparse
import os
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
COLORS = {'telemetry': '\033[36m', 'lidar': '\033[33m', 'camera': '\033[35m', 'stack': '\033[32m'}
RESET = '\033[0m'
STATUS_PERIOD = 5.0   # \r-style live status lines are throttled to this many seconds
_print_lock = threading.Lock()


def log(tag, msg):
    with _print_lock:
        print(f"{COLORS.get(tag, '')}[{tag:<9}]{RESET} {msg}", flush=True)


class Service:
    def __init__(self, name, cmd, stop_event):
        self.name, self.cmd, self.stop_event = name, cmd, stop_event
        self.proc = None
        self.thread = threading.Thread(target=self._run, daemon=True, name=name)

    def start(self):
        self.thread.start()

    def _pump(self, proc):
        """Forward child output; \\n lines immediately, \\r status lines throttled."""
        buf, last_status = b'', 0.0
        fd = proc.stdout
        while True:
            ch = fd.read1(4096) if hasattr(fd, 'read1') else fd.read(1)
            if not ch:
                break
            buf += ch
            while True:
                idx = [i for i in (buf.find(b'\n'), buf.find(b'\r')) if i >= 0]
                if not idx:
                    break
                i = min(idx)
                line, sep, buf = buf[:i].decode('utf-8', 'replace').strip(), buf[i:i + 1], buf[i + 1:]
                if not line:
                    continue
                if sep == b'\r':
                    if time.time() - last_status >= STATUS_PERIOD:
                        last_status = time.time()
                        log(self.name, line)
                else:
                    log(self.name, line)

    def _run(self):
        backoff = 2.0
        while not self.stop_event.is_set():
            started = time.time()
            log('stack', f"starting {self.name}: {' '.join(self.cmd[1:])}")
            self.proc = subprocess.Popen(
                self.cmd, cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                bufsize=0, preexec_fn=os.setsid)
            self._pump(self.proc)
            rc = self.proc.wait()
            if self.stop_event.is_set():
                break
            # reset back-off if it ran for a while, else grow it
            backoff = 2.0 if time.time() - started > 30 else min(backoff * 2, 15.0)
            log('stack', f"{self.name} exited (code {rc}); restarting in {backoff:.0f}s")
            self.stop_event.wait(backoff)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except ProcessLookupError:
                return
            try:
                self.proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)


def default_ip():
    c = os.environ.get('SSH_CLIENT')
    return c.split()[0] if c else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('target_ip', nargs='?', default=default_ip(), help='Laptop IP (default: SSH client IP)')
    ap.add_argument('--no-telemetry', action='store_true')
    ap.add_argument('--no-lidar', action='store_true')
    ap.add_argument('--no-camera', action='store_true')
    ap.add_argument('--nav-cmds', action='store_true',
                    help='Also accept Nav2 velocity commands on UDP 14553 (arms + GUIDED on first command!)')
    ap.add_argument('--pixhawk-port', default=None, help='Pixhawk serial port (default: auto /dev/pixhawk)')
    ap.add_argument('--lidar-port', default='/dev/ttyUSB0')
    ap.add_argument('--lidar-baud', type=int, default=115200)
    ap.add_argument('--camera-device', default='/dev/camera')
    ap.add_argument('--fps', type=float, default=20.0, help='Camera FPS limit')
    ap.add_argument('--quality', type=int, default=70, help='Camera JPEG quality')
    ap.add_argument('--rate', type=float, default=10.0, help='Telemetry JSON rate (Hz)')
    args = ap.parse_args()

    if not args.target_ip:
        ap.error('target_ip required (could not infer it from SSH_CLIENT)')

    py = [sys.executable, '-u']
    ip = args.target_ip
    cmds = {}
    if not args.no_telemetry:
        c = py + ['telemetry_streamer.py', '--target-ip', ip, '--rate', str(args.rate)]
        if args.pixhawk_port:
            c += ['--port', args.pixhawk_port]
        if args.nav_cmds:
            c += ['--velocity-port', '14553']
        cmds['telemetry'] = c
    elif args.nav_cmds:
        ap.error('--nav-cmds needs telemetry (it shares the Pixhawk link)')
    if not args.no_lidar:
        cmds['lidar'] = py + ['lidar_udp_streamer.py', '--target-ip', ip,
                              '--port', args.lidar_port, '--baud', str(args.lidar_baud)]
        if not os.path.exists(args.lidar_port):
            log('stack', f"WARNING: {args.lidar_port} not found - lidar will retry until it appears")
    if not args.no_camera:
        cmds['camera'] = py + ['camera_streamer.py', '--target-ip', ip, '--device', args.camera_device,
                               '--fps', str(args.fps), '--quality', str(args.quality)]
        if args.camera_device.startswith('/dev/') and not os.path.exists(args.camera_device):
            log('stack', f"WARNING: {args.camera_device} not found - camera will retry until it appears")
    if not cmds:
        ap.error('nothing to start')

    log('stack', f"streaming {', '.join(cmds)} -> {ip}   (Ctrl-C to stop)")
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())   # systemd / pkill -> clean shutdown
    services = [Service(n, c, stop) for n, c in cmds.items()]
    for s in services:
        s.start()
        time.sleep(1.0)   # stagger so the Pixhawk link comes up first

    try:
        while not stop.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    log('stack', 'stopping all streams...')
    stop.set()
    for s in services:
        s.stop()
    log('stack', 'done.')


if __name__ == '__main__':
    main()
