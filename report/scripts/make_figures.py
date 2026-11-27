#!/usr/bin/env python3
"""Regenerate the data figures of the report from files in the repository.

Inputs : raspberry_pi/plotjuggler_live.csv  (bench telemetry log, vehicle disarmed)
         report/data/seg_benchmark.csv       (from bench_segmentation.py)
Outputs: report/figures/*.pdf and report/data/bench_stats.txt
Usage  : python3 report/scripts/make_figures.py   (from the repository root)
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, 'report', 'figures')
DATA = os.path.join(ROOT, 'report', 'data')
C = ['#257de0', '#f47230', '#00b894']          # validated categorical slots 1-3
TXT, TXT2, GRID = '#191919', '#666666', '#e6e6e6'
G0 = 9.80665
ZUPT_WIN, ZUPT_A, ZUPT_G = 1.0, 15.0, 4.0       # mavlink_to_ros2_bridge defaults

plt.rcParams.update({
    'font.family': 'serif', 'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8.5,
    'axes.edgecolor': TXT2, 'axes.labelcolor': TXT, 'xtick.color': TXT2, 'ytick.color': TXT2,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.spines.top': False,
    'axes.spines.right': False, 'lines.linewidth': 1.0, 'legend.frameon': False,
    'legend.fontsize': 7, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02})

d = pd.read_csv(os.path.join(ROOT, 'raspberry_pi', 'plotjuggler_live.csv'))
t = d['timestamp'].to_numpy() - d['timestamp'].iloc[0]
stats = []
S = lambda *a: stats.append(' '.join(str(x) for x in a))
dt = np.diff(d['timestamp'])
S('samples', len(d), 'duration_s', round(t[-1], 2), 'median_rate_hz', round(1 / np.median(dt), 2),
  'dt_mean_ms', round(dt.mean() * 1e3, 1), 'dt_std_ms', round(dt.std() * 1e3, 1),
  'dt_max_ms', round(dt.max() * 1e3, 1))

# ---------------------------------------------------------------- Fig: sensor noise
fig, ax = plt.subplots(2, 2, figsize=(6.8, 4.3), sharex=True)
for i, k in enumerate('xyz'):
    s = d[f'imu/accel_{k}_mg']
    ax[0, 0].plot(t, s - s.mean(), color=C[i], label=f'{k}  (σ = {s.std():.1f} mg)')
    g = d[f'imu/gyro_{k}_dps']
    ax[0, 1].plot(t, g, color=C[i], label=f'{k}  (σ = {g.std():.2f} °/s)')
    S(f'accel_{k}_mean_mg', round(s.mean(), 1), 'std', round(s.std(), 2),
      f'gyro_{k}_mean_dps', round(g.mean(), 3), 'std', round(g.std(), 3))
ax[0, 0].set_ylabel('accel. − mean (mg)'); ax[0, 0].set_title('(a) Accelerometer')
ax[0, 1].set_ylabel('angular rate (°/s)'); ax[0, 1].set_title('(b) Gyroscope')
p = d['baro/pressure_hpa'].to_numpy()
alt = 44330.0 * (1 - (p / 1013.25) ** 0.190295)
ax[1, 0].plot(t, (alt - alt.mean()) * 100, color=C[0], label=f'σ = {alt.std()*100:.1f} cm')
ax[1, 0].set_ylabel('baro altitude − mean (cm)'); ax[1, 0].set_title('(c) Barometric altitude')
r = d['rangefinder/distance_m'] * 100
ax[1, 1].plot(t, r, color=C[0], drawstyle='steps-post', label=f'mean {r.mean():.1f} cm, σ = {r.std():.2f} cm')
ax[1, 1].set_ylabel('range (cm)'); ax[1, 1].set_title('(d) Downward rangefinder')
S('baro_pressure_std_hpa', round(p.std(), 4), 'baro_alt_std_cm', round(alt.std() * 100, 2),
  'baro_alt_pp_cm', round((alt.max() - alt.min()) * 100, 2))
S('range_mean_cm', round(r.mean(), 2), 'range_std_cm', round(r.std(), 3))
for a in ax[1]:
    a.set_xlabel('time (s)')
for a in ax.flat:
    ttl = a.get_title(); a.set_title(''); a.set_title(ttl, loc='left', pad=13)
    leg = a.legend(loc='lower right', bbox_to_anchor=(1.0, 0.98), ncol=3, handlelength=1.0,
                   columnspacing=0.8, fontsize=6.3)
    for ln in leg.get_lines():
        ln.set_linewidth(2.5)
fig.tight_layout(h_pad=1.2); fig.savefig(os.path.join(FIG, 'bench_sensor_noise.pdf')); plt.close(fig)

# ---------------------------------------------------------------- Fig: ZUPT detector
amag = np.sqrt(sum(d[f'imu/accel_{k}_mg'] ** 2 for k in 'xyz')).to_numpy()
gmag = np.sqrt(sum(d[f'imu/gyro_{k}_dps'] ** 2 for k in 'xyz')).to_numpy()
astd, gmean, stat = np.full(len(t), np.nan), np.full(len(t), np.nan), np.zeros(len(t), bool)
for i in range(len(t)):
    m = (t > t[i] - ZUPT_WIN) & (t <= t[i])
    if m.sum() >= 5 and t[m][-1] - t[m][0] >= 0.6 * ZUPT_WIN:
        astd[i], gmean[i] = amag[m].std(), gmag[m].mean()
        stat[i] = astd[i] < ZUPT_A and gmean[i] < ZUPT_G
valid = ~np.isnan(astd)
S('zupt_windows', int(valid.sum()), 'stationary_fraction', round(stat[valid].mean(), 4),
  'acc_std_max_mg', round(np.nanmax(astd), 2), 'acc_std_median_mg', round(np.nanmedian(astd), 2),
  'gyro_mean_max_dps', round(np.nanmax(gmean), 2), 'gyro_mean_median_dps', round(np.nanmedian(gmean), 2))
fig, ax = plt.subplots(1, 2, figsize=(6.8, 2.2), sharex=True)
ax[0].plot(t, astd, color=C[0], label='1 s std of |a|')
ax[0].axhline(ZUPT_A, color=TXT2, ls='--', lw=1, label=f'threshold {ZUPT_A:.0f} mg')
ax[0].set_ylabel('mg'); ax[0].set_title('(a) Specific-force variability')
ax[1].plot(t, gmean, color=C[1], label='1 s mean of |ω|')
ax[1].axhline(ZUPT_G, color=TXT2, ls='--', lw=1, label=f'threshold {ZUPT_G:.0f} °/s')
ax[1].set_ylabel('°/s'); ax[1].set_title('(b) Angular-rate magnitude')
for a in ax:
    a.set_xlabel('time (s)'); a.set_ylim(bottom=0); a.legend(loc='upper right')
fig.tight_layout(); fig.savefig(os.path.join(FIG, 'zupt_detector.pdf')); plt.close(fig)

# ---------------------------------------------------------------- Fig: IMU dead reckoning
def rot_nb(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
                     [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
                     [-sp, sr*cp, cr*cp]])

f_b = np.stack([d[f'imu/accel_{k}_mg'] for k in 'xyz'], 1) * 1e-3 * G0
eul = np.radians(d[['attitude/roll_deg', 'attitude/pitch_deg', 'attitude/yaw_deg']].to_numpy())
a_n = np.array([rot_nb(*e) @ f for e, f in zip(eul, f_b)]) + np.array([0, 0, G0])

def integrate(acc):
    v, p = np.zeros_like(acc), np.zeros_like(acc)
    for i in range(1, len(acc)):
        h = t[i] - t[i - 1]
        v[i] = v[i - 1] + 0.5 * (acc[i] + acc[i - 1]) * h
        p[i] = p[i - 1] + 0.5 * (v[i] + v[i - 1]) * h
    return np.linalg.norm(p[:, :2], axis=1), np.linalg.norm(p, axis=1)

raw_h, raw_3 = integrate(a_n)
n1 = np.searchsorted(t, 1.0)          # "calibrate" bias on the first second only
cal_h, cal_3 = integrate(a_n - a_n[:n1].mean(0))
S('mean_residual_accel_ned_mps2', np.round(a_n.mean(0), 4).tolist())
S('dr_raw_horiz_err_end_m', round(raw_h[-1], 2), 'dr_raw_3d_err_end_m', round(raw_3[-1], 2))
S('dr_cal1s_horiz_err_end_m', round(cal_h[-1], 3), 'dr_cal1s_3d_err_end_m', round(cal_3[-1], 3))
for T in (5, 10, 20):
    i = np.searchsorted(t, T)
    S(f'dr_at_{T}s raw_h', round(raw_h[i], 3), 'cal_h', round(cal_h[i], 3))
fig, ax = plt.subplots(1, 2, figsize=(6.8, 2.3))
ax[0].plot(t, raw_h, color=C[0], label='uncompensated (horizontal)')
ax[0].set_title('(a) No bias compensation'); ax[0].set_ylabel('position error (m)')
ax[1].plot(t, cal_h, color=C[1], label='bias from first 1 s removed (horizontal)')
ax[1].set_title('(b) Static bias removed')
for a in ax:
    a.set_xlabel('time (s)'); a.legend(loc='upper left')
fig.tight_layout(); fig.savefig(os.path.join(FIG, 'imu_dead_reckoning.pdf')); plt.close(fig)

# ---------------------------------------------------------------- Fig: segmentation timing
b = pd.read_csv(os.path.join(DATA, 'seg_benchmark.csv'))
fig, ax = plt.subplots(figsize=(4.6, 1.7))
y = np.arange(len(b))[::-1]
other = b['mean_ms'] - b['yolo_ms'] - b['sam_ms']
ax.barh(y, b['yolo_ms'], color=C[0], height=0.55, label='YOLOv8n-seg', edgecolor='white', lw=1)
ax.barh(y, b['sam_ms'], left=b['yolo_ms'], color=C[1], height=0.55, label='SAM 2.1-t', edgecolor='white', lw=1)
ax.barh(y, other, left=b['yolo_ms'] + b['sam_ms'], color=C[2], height=0.55, label='fusion + overhead', edgecolor='white', lw=1)
for yi, m, f in zip(y, b['mean_ms'], b['fps']):
    ax.text(m + 0.8, yi, f'{m:.1f} ms ({f:.0f} FPS)', va='center', fontsize=7, color=TXT)
ax.set_yticks(y, b['config']); ax.set_xlabel('mean per-frame latency (ms), 640×480 input')
ax.set_xlim(0, b['mean_ms'].max() * 1.45); ax.grid(axis='y', visible=False)
ax.legend(loc='lower center', bbox_to_anchor=(0.4, 1.0), ncol=3)
fig.savefig(os.path.join(FIG, 'seg_benchmark.pdf')); plt.close(fig)

with open(os.path.join(DATA, 'bench_stats.txt'), 'w') as f:
    f.write('\n'.join(stats) + '\n')
print('\n'.join(stats))
