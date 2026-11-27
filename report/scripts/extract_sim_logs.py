#!/usr/bin/env python3
"""Extract the PX4 SITL (Gazebo Harmonic) flight logs of the project's simulation runs
into small CSV files under report/data/sim/.

    python3 report/scripts/extract_sim_logs.py \
        --ulog-dir ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-11-20 \
        --ros-log-dir ~/.ros/log

The raw ULog files (0.7-170 MB each) stay outside the repository; this script keeps only
what the report plots:

  data/sim/sim_runs.csv          one row per flight (>= 30 s in OFFBOARD): world, durations,
                                 path, speeds, height hold, velocity tracking, LiDAR
                                 clearance, estimator error, incident, explore_lite outcome
  data/sim/run_<HH_MM_SS>.csv    5 Hz time series of each flight
  data/sim/scan_<HH_MM_SS>.csv   LiDAR returns of the flight projected with the Gazebo
                                 ground-truth pose, 0.10 m grid cells with hit counts

Everything comes from PX4's own topics: vehicle_local_position_groundtruth and
vehicle_attitude_groundtruth (Gazebo truth), vehicle_local_position (EKF2 estimate),
trajectory_setpoint (the commands x500_px4_bridge sent in OFFBOARD), obstacle_distance
(the 2D LiDAR scan as 72 bins of 5 deg, body FRD, starting at -135 deg) and vehicle_status.

In these runs EKF2 fused the simulated GPS (EKF2_GPS_CTRL=7), so "est_err" is the error
of a GPS-aided estimate, not of a GPS-denied one. Statistics are computed from 15 s after
OFFBOARD starts until OFFBOARD ends or 2 s before the first incident (the vehicle falls
below 0.5 m while commanded to fly higher), whichever is first; the time series keep the whole flight.
"""
import argparse
import csv
import datetime as dt
import glob
import os
import re
import warnings

import numpy as np
from pyulog import ULog

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'data', 'sim')
TOPICS = ['vehicle_local_position_groundtruth', 'vehicle_attitude_groundtruth',
          'vehicle_local_position', 'trajectory_setpoint', 'obstacle_distance',
          'vehicle_status']
NAV_OFFBOARD, ARMED = 14, 2
# Spawn points in PX4 NED world coordinates (north = Gazebo y, east = Gazebo x).
SPAWNS = {'office': (-4.5, 8.0),             # sim_office.launch.py LOBBY_POSE
          'baylands': (93.2, -80.7),         # sim_baylands.launch.py default (tree grove)
          'baylands-origin': (0.0, 0.0)}     # Baylands with PX4's open-field spawn (earlier runs)
SAFETY_STOP, SAFETY_PUSH = 0.65, 0.45        # x500_px4_bridge LiDAR safety distances (m)
MIN_OFFBOARD = 30.0                          # skip flights shorter than this in OFFBOARD (s)
SETTLE = 15.0                                # s after OFFBOARD starts before statistics
INCIDENT_HEIGHT = 0.5                        # m: falling below this while commanded higher = incident
GRID = 0.10                                  # m, LiDAR map cell


def interp(t_src, y, t):
    return np.interp(t, t_src, y, left=np.nan, right=np.nan)


def explore_runs(ros_log_dir):
    """(start, stop, reason) in UTC epoch seconds for every explore_lite log."""
    runs = []
    for f in glob.glob(os.path.join(ros_log_dir, 'explore_*.log')):
        start = stop = None
        reason = 'stopped by operator'
        for line in open(f, errors='replace'):
            m = re.match(r'\[\w+\] \[([0-9.]+)\]', line)
            if not m:
                continue
            ts = float(m.group(1))
            if 'Getting initial pose' in line:
                start = ts
            if 'Exploration stopped' in line:
                stop = ts
            if 'All frontiers traversed' in line:
                reason = 'all frontiers traversed/tried'
            if 'No frontiers found' in line:
                reason = 'no frontiers left'
        if start:
            runs.append((start, stop, reason))
    return runs


def fmt(x, p=3):
    return '' if x is None or not np.isfinite(x) else f'{x:.{p}f}'


def analyse(path, explore):
    u = ULog(path, TOPICS)
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    if not all(k in d for k in TOPICS):
        return None
    t0 = int(u.start_timestamp)
    sec = lambda a: (a['timestamp'].astype(np.int64) - t0) / 1e6
    gt, att, est, sp, od, st = (d[k] for k in TOPICS)

    ts = sec(st)
    offb = (st['nav_state'] == NAV_OFFBOARD) & (st['arming_state'] == ARMED)
    if offb.sum() < 2 or ts[offb][-1] - ts[offb][0] < MIN_OFFBOARD:
        return None
    t_on, t_off = ts[offb][0], ts[offb][-1]

    # ---- 5 Hz time series ------------------------------------------------------------
    t = np.arange(np.ceil(max(sec(gt)[0], 0)), sec(gt)[-1], 0.2)
    g = {k: interp(sec(gt), gt[k], t) for k in ('x', 'y', 'z', 'vx', 'vy')}
    e = {k: interp(sec(est), est[k], t) for k in ('x', 'y', 'vx', 'vy')}
    s = {k: interp(sec(sp), sp[k], t) for k in ('velocity[0]', 'velocity[1]', 'position[2]')}
    nav = np.interp(t, ts, offb.astype(float)) > 0.5

    q = [att[f'q[{i}]'] for i in range(4)]
    yaw_att = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))

    # LiDAR: nearest valid return per scan (cm -> m)
    D = np.stack([od[f'distances[{i}]'] for i in range(72)], 1).astype(float)
    valid = (D >= od['min_distance'][:, None]) & (D <= od['max_distance'][:, None])
    Dm = np.where(valid, D, np.nan) / 100.0
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        nearest = np.nanmin(Dm, 1)
    lid = interp(sec(od), np.nan_to_num(nearest, nan=np.inf), t)
    lid[np.isinf(lid)] = np.nan

    spawn = min(SPAWNS, key=lambda w: np.hypot(g['x'][0] - SPAWNS[w][0], g['y'][0] - SPAWNS[w][1]))
    world = 'office' if spawn == 'office' else 'baylands'

    # EKF origin is the spawn point: express the estimate in world coordinates
    i0 = np.flatnonzero(np.isfinite(e['x']) & np.isfinite(g['x']))[0]
    en, ee = e['x'] + g['x'][i0] - e['x'][i0], e['y'] + g['y'][i0] - e['y'][i0]

    sp_alt = -s['position[2]']
    hgt = -(g['z'] - g['z'][0])                              # height above take-off floor
    speed = np.hypot(g['vx'], g['vy'])
    sp_speed = np.hypot(s['velocity[0]'], s['velocity[1]'])
    trk = np.hypot(s['velocity[0]'] - e['vx'], s['velocity[1]'] - e['vy'])
    pos_err = np.hypot(en - g['x'], ee - g['y'])

    # first incident: the vehicle falls below INCIDENT_HEIGHT while commanded to fly higher
    flying = nav & (t > t_on + SETTLE)
    drop = flying & np.isfinite(sp_alt) & (sp_alt > 2 * INCIDENT_HEIGHT) & (hgt < INCIDENT_HEIGHT)
    t_inc = t[drop][0] if drop.any() else None
    end = t_inc - 2.0 if t_inc is not None else np.inf       # stats stop 2 s before it
    w = flying & (t < end) & np.isfinite(g['x'])
    altw = w & np.isfinite(sp_alt)
    lidw = w & np.isfinite(lid)
    nav_pre = nav & (t < end)
    path_len = np.nansum(np.hypot(np.diff(g['x'][nav_pre]), np.diff(g['y'][nav_pre])))

    epoch = dt.datetime.strptime(os.path.basename(os.path.dirname(path)) + ' ' +
                                 os.path.basename(path)[:8], '%Y-%m-%d %H_%M_%S'
                                 ).replace(tzinfo=dt.timezone.utc).timestamp()
    ex = [r for r in explore if epoch + t_on - 60 < r[0] < epoch + t_off]
    ex_dur = ex_reason = ''
    if ex:
        start, stop, ex_reason = ex[0]
        ex_dur = f'{stop - start:.0f}' if stop else ''

    pct = lambda a, p: np.nanpercentile(a, p) if a.size else np.nan
    name = os.path.basename(path)[:8]
    # where the fall began: last sample before it still within 0.1 m of the commanded height
    k_inc = None
    if t_inc is not None:
        held = np.flatnonzero((t < t_inc) & (sp_alt - hgt < 0.1))
        k_inc = held[-1] if held.size else np.searchsorted(t, t_inc)
    row = dict(
        run=name, world=world, spawn=spawn,
        offboard_s=f'{t_off - t_on:.0f}', analysed_s=f'{w.sum() * 0.2:.0f}',
        path_m=f'{path_len:.1f}',
        alt_target_m=fmt(np.nanmedian(sp_alt[altw]), 2) if altw.any() else '',
        alt_err_rms_m=fmt(np.sqrt(np.nanmean((hgt - sp_alt)[altw] ** 2))) if altw.any() else '',
        alt_err_max_m=fmt(np.nanmax(np.abs(hgt - sp_alt)[altw])) if altw.any() else '',
        speed_mean=fmt(np.nanmean(speed[w])), speed_p95=fmt(pct(speed[w], 95)),
        speed_max=fmt(np.nanmax(speed[w])), cmd_speed_max=fmt(np.nanmax(sp_speed[w])),
        vel_track_rms=fmt(np.sqrt(np.nanmean(trk[w] ** 2))),
        lidar_min_m=fmt(np.nanmin(lid[lidw]), 2) if lidw.any() else '',
        lidar_p5_m=fmt(pct(lid[lidw], 5), 2) if lidw.any() else '',
        frac_below_stop=fmt(np.mean(lid[lidw] < SAFETY_STOP), 4) if lidw.any() else '',
        frac_below_push=fmt(np.mean(lid[lidw] < SAFETY_PUSH), 4) if lidw.any() else '',
        est_err_rms_m=fmt(np.sqrt(np.nanmean(pos_err[w] ** 2))),
        est_err_max_m=fmt(np.nanmax(pos_err[w])),
        incident_t=fmt(t[k_inc] - t_on, 1) if t_inc is not None else '',
        incident_n=fmt(g['x'][k_inc], 2) if k_inc is not None else '',
        incident_e=fmt(g['y'][k_inc], 2) if k_inc is not None else '',
        incident_lidar_min_m=fmt(np.nanmin(lid[(t > t[k_inc] - 3) & (t <= t[k_inc] + 0.4)]), 2)
        if t_inc is not None else '',
        explore_s=ex_dur, explore_end=ex_reason,
    )
    series = dict(t=t - t_on, offboard=nav.astype(int), gt_n=g['x'], gt_e=g['y'], height=hgt,
                  est_n=en, est_e=ee, vn=g['vx'], ve=g['vy'], sp_vn=s['velocity[0]'],
                  sp_ve=s['velocity[1]'], sp_alt=sp_alt, est_vn=e['vx'], est_ve=e['vy'],
                  lidar_min=lid, est_err=pos_err)

    # ---- LiDAR map: every return before the incident, at the ground-truth pose ---------
    to = sec(od)
    keep = (to > t_on) & (to < min(t_off, end))
    pn = np.interp(to, sec(gt), gt['x'])[keep]
    pe = np.interp(to, sec(gt), gt['y'])[keep]
    yaw = np.unwrap(yaw_att)
    yaw = np.interp(to, sec(att), yaw)[keep]
    ang = np.deg2rad(od['angle_offset'][keep, None] + od['increment'][keep, None] * np.arange(72))
    r = Dm[keep]
    b = yaw[:, None] + ang                                   # FRD: clockwise from north
    xn, xe = pn[:, None] + r * np.cos(b), pe[:, None] + r * np.sin(b)
    ok = np.isfinite(r) & (r < 12.0)
    cells, counts = np.unique(np.stack([np.round(xn[ok] / GRID), np.round(xe[ok] / GRID)], 1)
                              .astype(np.int64), axis=0, return_counts=True)
    scan = (cells * GRID, counts)
    return row, series, scan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ulog-dir', default=os.path.expanduser(
        '~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-11-20'))
    ap.add_argument('--ros-log-dir', default=os.path.expanduser('~/.ros/log'))
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    explore = explore_runs(a.ros_log_dir)
    rows = []
    for f in sorted(glob.glob(os.path.join(a.ulog_dir, '*.ulg'))):
        res = analyse(f, explore)
        if res is None:
            print(f'skip {os.path.basename(f)}')
            continue
        row, ser, (cells, counts) = res
        rows.append(row)
        keys = list(ser)
        with open(os.path.join(OUT, f'run_{row["run"]}.csv'), 'w', newline='') as fh:
            wr = csv.writer(fh)
            wr.writerow(keys)
            for i in range(len(ser['t'])):
                wr.writerow([int(ser[k][i]) if k == 'offboard' else fmt(ser[k][i]) for k in keys])
        with open(os.path.join(OUT, f'scan_{row["run"]}.csv'), 'w', newline='') as fh:
            wr = csv.writer(fh)
            wr.writerow(['n', 'e', 'hits'])
            wr.writerows([[f'{c[0]:.1f}', f'{c[1]:.1f}', n] for c, n in zip(cells, counts)])
        print(row['run'], row['world'], row['analysed_s'], 's', row['incident_t'], row['explore_end'])
    with open(os.path.join(OUT, 'sim_runs.csv'), 'w', newline='') as fh:
        wr = csv.DictWriter(fh, fieldnames=list(rows[0]))
        wr.writeheader()
        wr.writerows(rows)
    print(f'{len(rows)} runs -> {os.path.normpath(OUT)}')


if __name__ == '__main__':
    main()
