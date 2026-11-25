#!/usr/bin/env python3
"""
start_streams.py - ONE command on the Raspberry Pi to stream everything to the laptop.

    python3 start_streams.py 10.184.3.231

Starts and supervises (auto-restart with back-off if one crashes):
    telemetry : telemetry_streamer.py   -> UDP 14551 JSON, 14550 MAVLink  (owns the Pixhawk link;
                                           also serves arm/mode commands on UDP 14552)
    lidar     : lidar_udp_streamer.py   -> UDP 14552 (compact binary scans)
    camera    : camera_streamer.py      -> TCP 5000 (laptop must be listening; it reconnects)
    nav cmds  : optional (--nav-cmds), Nav2 velocity listener on UDP 14553, sharing the
                telemetry process's Pixhawk connection (no serial-port conflict).

Ctrl-C stops everything. Use --no-lidar / --no-camera / --no-telemetry to skip a stream.
If the target IP is omitted, the SSH client IP is used when run over ssh, otherwise
the default gateway (the laptop, when the Pi is on the laptop's hotspot).

Latency: Wi-Fi power saving is switched OFF on the interface that routes to the laptop
(re-checked every 30 s, since a reconnect can turn it back on). With it on, the Pi's radio
sleeps between beacons and buffers incoming packets (Nav2 velocity commands, TCP ACKs
pacing the camera) for tens to hundreds of ms. Needs root or passwordless `sudo iw`;
otherwise a warning with the manual command is printed. --keep-powersave skips this.

Clock sync: the laptop's latency metrics compare Pi timestamps with the laptop clock, so the
two clocks must agree. On start, the laptop is added to chrony as the preferred, trusted time
source and the clock is stepped onto it; on exit the source is removed again, so the Pi goes
back to its normal NTP servers. Needs chrony on both machines (laptop as server:
scripts/setup_laptop_timeserver.sh) and passwordless `sudo chronyc` on the Pi
(setup_pi.sh). --no-time-sync skips this.
"""
import argparse
import os
import shutil
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
    def __init__(self, name, cmd, stop_event, env=None):
        self.name, self.cmd, self.stop_event = name, cmd, stop_event
        self.env = env
        self.proc = None
        self.thread = threading.Thread(target=self._run, daemon=True, name=name)

    def start(self):
        self.thread.start()

    def _pump(self, proc):
        """Forward child output; \\n lines immediately, \\r status lines throttled."""
        buf, last_status = b'', 0.0
        fd = proc.stdout
        while True:
            # bufsize=0 gives a raw FileIO (no read1); its read(n) already returns
            # whatever is available, so do not fall back to one byte per syscall.
            ch = fd.read1(4096) if hasattr(fd, 'read1') else fd.read(4096)
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
                bufsize=0, preexec_fn=os.setsid, env=self.env)
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


def _route_dev(target_ip):
    """Network interface the kernel uses to reach target_ip (e.g. 'wlan0'), or None."""
    try:
        out = subprocess.run(['ip', 'route', 'get', target_ip], capture_output=True,
                             text=True, timeout=2).stdout.split()
        return out[out.index('dev') + 1] if 'dev' in out else None
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return None


def _iw_path():
    """`iw` lives in /usr/sbin on Debian, which is not on a normal user's PATH."""
    return shutil.which('iw') or next(
        (p for p in ('/usr/sbin/iw', '/sbin/iw') if os.access(p, os.X_OK)), None)


def ensure_wifi_powersave_off(target_ip, quiet=False):
    """Disable 802.11 power saving on the interface routing to target_ip.
    Returns True if it is off (or the route is not Wi-Fi)."""
    dev = _route_dev(target_ip)
    if not dev or not os.path.isdir(f'/sys/class/net/{dev}/wireless'):
        return True                 # unknown route or wired / USB-tether link
    iw = _iw_path()
    try:
        if not iw:
            raise FileNotFoundError('iw')
        st = subprocess.run([iw, 'dev', dev, 'get', 'power_save'], capture_output=True,
                            text=True, timeout=2)
    except (OSError, subprocess.SubprocessError):
        if not quiet:
            log('stack', "WARNING: 'iw' not found; cannot check Wi-Fi power saving "
                         "(sudo apt install iw)")
        return False
    if st.returncode != 0:
        if not quiet:
            log('stack', f"WARNING: could not read Wi-Fi power saving on {dev}: "
                         f"{st.stderr.strip()}")
        return False
    if 'off' in st.stdout.lower():
        return True
    cmd = [iw, 'dev', dev, 'set', 'power_save', 'off']
    if os.geteuid() != 0:
        cmd = ['sudo', '-n'] + cmd
    try:
        ok = subprocess.run(cmd, capture_output=True, timeout=3).returncode == 0
    except (OSError, subprocess.SubprocessError):
        ok = False
    if ok:
        log('stack', f"Wi-Fi power saving disabled on {dev} (lower, steadier latency)")
    elif not quiet:
        log('stack', f"WARNING: Wi-Fi power saving is ON for {dev} and could not be disabled "
                     f"(adds 10-200 ms per packet). Run: sudo iw dev {dev} set power_save off")
    return ok


def _chronyc_path():
    return shutil.which('chronyc') or next(
        (p for p in ('/usr/bin/chronyc', '/bin/chronyc') if os.access(p, os.X_OK)), None)


def _chronyc_root(chronyc, *args):
    """Run a privileged chronyc command (root only: /run/chrony is not user-accessible)."""
    cmd = [chronyc] + list(args)
    if os.geteuid() != 0:
        cmd = ['sudo', '-n'] + cmd
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return r.returncode == 0, (r.stdout + r.stderr).strip()
    except (OSError, subprocess.SubprocessError) as e:
        return False, str(e)


def _chrony_source(chronyc, ip):
    """chrony's view of source `ip`: (state, reach, offset_s) or None.
    state: '*' selected, '+' combined, '?' unreachable/unknown, 'x' falseticker, ..."""
    try:
        out = subprocess.run([chronyc, '-c', '-n', 'sources'], capture_output=True,
                             text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for line in out.splitlines():
        f = line.split(',')
        if len(f) >= 8 and f[2] == ip:
            try:
                return f[1], int(f[5], 8), float(f[7])
            except ValueError:
                return f[1], 0, float('nan')
    return None


def sync_clock_to(ip, timeout=20.0):
    """Make the laptop the Pi's time source (chrony) and step the clock onto it.
    Returns True when the Pi is synced to the laptop."""
    chronyc = _chronyc_path()
    if not chronyc:
        log('stack', 'WARNING: chrony not installed; Pi/laptop clocks may differ by 100+ ms '
                     '(latency metrics will be off). Run setup_pi.sh')
        return False
    # prefer: use only the laptop while it is reachable. trust: never reject it because it
    # disagrees with the internet servers (they are far away and jittery over Wi-Fi).
    # minpoll 0 / maxpoll 4: poll every 1-16 s, so Wi-Fi jitter is averaged out quickly.
    ok, msg = _chronyc_root(chronyc, 'add', 'server', ip, 'iburst', 'minpoll', '0',
                            'maxpoll', '4', 'prefer', 'trust')
    if not ok and 'already' not in msg.lower():
        log('stack', f"WARNING: could not add the laptop as time source ({msg or 'sudo refused'}). "
                     "Run setup_pi.sh once (allows passwordless `sudo chronyc`)")
        return False
    t0 = time.monotonic()
    src = None
    while time.monotonic() - t0 < timeout:
        src = _chrony_source(chronyc, ip)
        if src and src[0] == '*':
            break
        time.sleep(0.5)
    if not src or src[0] != '*':
        reach = src[1] if src else 0
        hint = ('the laptop does not answer NTP: run scripts/setup_laptop_timeserver.sh '
                'on it (chrony server + UDP 123 open)') if not reach else \
            f"chrony has not selected it yet (state '{src[0]}'); it keeps trying in the background"
        log('stack', f'WARNING: clock not synced to laptop {ip}: {hint}')
        return False
    _chronyc_root(chronyc, 'makestep')            # jump now instead of slewing for minutes
    time.sleep(1.0)
    src = _chrony_source(chronyc, ip) or src
    log('stack', f'clock synced to laptop {ip} (offset {src[2] * 1e3:+.1f} ms, '
                 f'took {time.monotonic() - t0:.1f} s)')
    return True


def unsync_clock_from(ip):
    """Remove the laptop time source again (the Pi falls back to its normal NTP servers)."""
    chronyc = _chronyc_path()
    if chronyc:
        _chronyc_root(chronyc, 'delete', ip)


def default_ip():
    """Laptop IP: the SSH client when run over ssh, else the default gateway (the laptop
    when the Pi is on the laptop's hotspot, e.g. 10.42.0.1 with NetworkManager sharing)."""
    c = os.environ.get('SSH_CLIENT')
    if c:
        return c.split()[0]
    try:
        out = subprocess.run(['ip', '-4', 'route', 'show', 'default'], capture_output=True,
                             text=True, timeout=2).stdout.split()
        return out[out.index('via') + 1] if 'via' in out else None
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('target_ip', nargs='?', default=None,
                    help='Laptop IP (default: SSH client IP, else the default gateway)')
    ap.add_argument('--no-telemetry', action='store_true')
    ap.add_argument('--no-lidar', action='store_true')
    ap.add_argument('--no-camera', action='store_true')
    ap.add_argument('--nav-cmds', action='store_true',
                    help='Also accept Nav2 velocity commands on UDP 14553 (forwarded only while armed in GUIDED)')
    ap.add_argument('--pixhawk-port', default=None, help='Pixhawk serial port (default: auto /dev/pixhawk)')
    ap.add_argument('--lidar-port', default='/dev/ttyUSB0')
    ap.add_argument('--lidar-baud', type=int, default=115200)
    ap.add_argument('--camera-device', default='/dev/camera')
    ap.add_argument('--fps', type=float, default=20.0, help='Camera FPS limit')
    ap.add_argument('--quality', type=int, default=70, help='Camera JPEG quality')
    ap.add_argument('--camera-size', default='640x480',
                    help='Camera resolution WxH (e.g. 424x240 for less Wi-Fi load)')
    ap.add_argument('--camera-passthrough', action='store_true',
                    help="Forward the camera's MJPEG untouched (no re-encode on the Pi)")
    ap.add_argument('--camera-usb-reset', action='store_true',
                    help='Let the camera streamer reset the Pi 4 USB controller when the camera '
                         'vanishes (WARNING: also drops the Pixhawk/LiDAR USB links)')
    ap.add_argument('--no-time-sync', action='store_true',
                    help="Don't sync the Pi clock to the laptop (chrony)")
    ap.add_argument('--keep-powersave', action='store_true',
                    help='Do not disable Wi-Fi power saving (it adds latency)')
    ap.add_argument('--rate', type=float, default=30.0,
                    help='Max telemetry JSON rate (Hz); sent when fresh Pixhawk nav data arrives')
    args = ap.parse_args()

    if not args.target_ip:
        args.target_ip = default_ip()
        if not args.target_ip:
            ap.error('target_ip required (could not infer it from SSH_CLIENT or the default route)')
        log('stack', f"laptop IP not given; using {args.target_ip} "
                     f"({'SSH client' if os.environ.get('SSH_CLIENT') else 'default gateway'})")

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
        cw, ch = args.camera_size.lower().split('x')
        cmds['camera'] += ['--width', cw, '--height', ch]
        if args.camera_passthrough:
            cmds['camera'].append('--passthrough')
        if args.camera_usb_reset:
            cmds['camera'].append('--usb-reset')
        if args.camera_device.startswith('/dev/') and not os.path.exists(args.camera_device):
            log('stack', f"WARNING: {args.camera_device} not found - camera will retry until it appears")
    if not cmds:
        ap.error('nothing to start')

    if not args.keep_powersave:
        ensure_wifi_powersave_off(ip)
    if not args.no_time_sync:   # in the background: streaming starts immediately
        threading.Thread(target=sync_clock_to, args=(ip,), daemon=True, name='timesync').start()
    log('stack', f"streaming {', '.join(cmds)} -> {ip}   (Ctrl-C to stop)")
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())   # systemd / pkill -> clean shutdown
    envs = {}
    if 'lidar' in cmds:   # Pixhawk auto-detection must never probe the LiDAR's serial port
        envs['telemetry'] = dict(os.environ, GPS_DENIED_EXCLUDE_PORTS=args.lidar_port)
    services = [Service(n, c, stop, envs.get(n)) for n, c in cmds.items()]
    for s in services:
        s.start()
        time.sleep(1.0)   # stagger so the Pixhawk link comes up first

    try:
        last_ps_check = time.monotonic()
        while not stop.is_set():
            time.sleep(0.5)
            if not args.keep_powersave and time.monotonic() - last_ps_check > 30.0:
                last_ps_check = time.monotonic()
                ensure_wifi_powersave_off(ip, quiet=True)   # reconnects can re-enable it
    except KeyboardInterrupt:
        pass
    log('stack', 'stopping all streams...')
    stop.set()
    for s in services:
        s.stop()
    if not args.no_time_sync:
        unsync_clock_from(ip)
    log('stack', 'done.')


if __name__ == '__main__':
    main()
