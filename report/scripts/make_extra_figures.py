#!/usr/bin/env python3
"""Summary figures built from numbers and data already in the repository.

Inputs : report/data/bench_telemetry_2026-11-15.csv  (stationary bench log)
         constants below, each traceable to a table or section of the report
Outputs: report/figures/link_latency.pdf, packet_delivery.pdf,
         drift_comparison.pdf, kpi_status.pdf
Usage  : python3 report/scripts/make_extra_figures.py   (from the repository root)
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
C = ['#257de0', '#f47230', '#00b894']          # same categorical slots as make_figures.py
TXT, TXT2, GRID = '#191919', '#666666', '#e6e6e6'
G0 = 9.80665

plt.rcParams.update({
    'font.family': 'serif', 'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8.5,
    'axes.edgecolor': TXT2, 'axes.labelcolor': TXT, 'xtick.color': TXT2, 'ytick.color': TXT2,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.spines.top': False,
    'axes.spines.right': False, 'lines.linewidth': 1.0, 'legend.frameon': False,
    'legend.fontsize': 7, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02})


def save(fig, name):
    fig.savefig(os.path.join(FIG, name)); plt.close(fig)


# ------------------------------------------------ Fig: link latency before/after (Table 'latency')
paths = ['Pixhawk → telemetry sent', 'telemetry → /odom', '/cmd_vel → Pixhawk']
before = np.array([28.0, 20.0, 54.0])
after = np.array([5.0, 2.0, 1.4])
fig, ax = plt.subplots(figsize=(4.8, 1.75))
y = np.arange(len(paths))[::-1]
h = 0.34
ax.barh(y + h / 2 + 0.01, before, height=h, color=C[1], label='before', edgecolor='white', lw=1)
ax.barh(y - h / 2 - 0.01, after, height=h, color=C[0], label='after', edgecolor='white', lw=1)
for yi, b, a in zip(y, before, after):
    ax.text(b + 0.8, yi + h / 2, f'{b:g} ms', va='center', fontsize=7, color=TXT)
    ax.text(a + 0.8, yi - h / 2, f'{a:g} ms  ({b / a:.0f}× faster)', va='center', fontsize=7, color=TXT)
ax.set_yticks(y); ax.set_yticklabels(paths)
ax.set_xlabel('software-path latency (ms), single host, Wi-Fi air time excluded')
ax.set_xlim(0, 70); ax.grid(axis='y', visible=False)
ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.18), ncol=2)
save(fig, 'link_latency.pdf')

# ------------------------------------------------ Fig: datagram delivery vs fragment loss (eq. frag)
p = np.linspace(0, 0.2, 201)
fig, ax = plt.subplots(figsize=(3.4, 2.1))
for n, col, lab in [(1, C[0], 'binary scan, 1 datagram (1094 B)'),
                    (3, C[1], 'JSON scan, 3 fragments (≈4 kB)')]:
    ax.plot(p * 100, (1 - p) ** n, color=col, lw=2, label=lab)
    ax.plot([5], [(0.95) ** n], 'o', ms=5, color=col, mec='white', mew=1.5)
    ax.annotate(f'{0.95 ** n:.2f}', (5, 0.95 ** n), xytext=(6, 5),
                textcoords='offset points', fontsize=7, color=TXT)
ax.axvline(5, color=TXT2, ls='--', lw=0.8)
ax.set_xlabel('fragment loss probability p (%)'); ax.set_ylabel('scan delivery probability')
ax.set_xlim(0, 20); ax.set_ylim(0.3, 1.01)
ax.legend(loc='lower left')
save(fig, 'packet_delivery.pdf')

# ------------------------------------------------ Fig: drift comparison (sec. val-dr, val-hwdemo)
d = pd.read_csv(os.path.join(DATA, 'bench_telemetry_2026-11-15.csv'))
t = d['timestamp'].to_numpy() - d['timestamp'].iloc[0]


def rot_nb(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
                     [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
                     [-sp, sr*cp, cr*cp]])


f_b = np.stack([d[f'imu/accel_{k}_mg'] for k in 'xyz'], 1) * 1e-3 * G0
eul = np.radians(d[['attitude/roll_deg', 'attitude/pitch_deg', 'attitude/yaw_deg']].to_numpy())
a_n = np.array([rot_nb(*e) @ f for e, f in zip(eul, f_b)]) + np.array([0, 0, G0])


def integrate(acc):
    v, q = np.zeros_like(acc), np.zeros_like(acc)
    for i in range(1, len(acc)):
        dt = t[i] - t[i - 1]
        v[i] = v[i - 1] + 0.5 * (acc[i] + acc[i - 1]) * dt
        q[i] = q[i - 1] + 0.5 * (v[i] + v[i - 1]) * dt
    return np.linalg.norm(q[:, :2], axis=1)


raw_h = integrate(a_n)
n1 = np.searchsorted(t, 1.0)
cal_h = integrate(a_n - a_n[:n1].mean(0))
m = t >= 1.0
fig, ax = plt.subplots(figsize=(4.2, 2.6))
ax.plot(t[m], raw_h[m], color=C[1], lw=2, label='IMU only, uncompensated (measured)')
ax.plot(t[m], cal_h[m], color=C[0], lw=2, label='IMU only, static bias removed (measured)')
tt = np.logspace(0, np.log10(600), 100)
ax.plot(tt, 2.0 * tt / 60, color=TXT2, ls=':', lw=1.2, label='typical GNSS-free fallback, 2 m/min (KPI table)')
ax.plot([420], [1.6], 'o', ms=8, color=C[2], mec='white', mew=1.5, zorder=5,
        label='this vehicle: flow + rangefinder hover, 7 min')
ax.annotate('1.6 m', (420, 1.6), xytext=(-30, 6), textcoords='offset points', fontsize=7, color=TXT)
ax.plot([600], [0.05], marker='*', ms=10, color=TXT, ls='none', label='KPI: ±5 cm over 10 min')
ax.set_xscale('log'); ax.set_yscale('log')
ax.set_xlim(1, 900); ax.set_ylim(0.01, 300)
ax.set_xlabel('time (s, log scale)'); ax.set_ylabel('horizontal error (m, log scale)')
ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=7)
save(fig, 'drift_comparison.pdf')

# ------------------------------------------------ Fig: KPI status (Table 'kpistatus')
# achieved / target, oriented so that > 1 means the target is met
rows = [
    ('Perception rate (32 FPS vs 30 Hz)', 32 / 30),
    ('Vision rate (camera 15 Hz vs 30 Hz)', 15 / 30),
    ('Endurance (target battery 10 vs 30 min)', 10 / 30),
    ('State rate (/odom 30 Hz vs 100 Hz)', 30 / 100),
    ('Hover drift (5 cm target vs 1.6 m in 7 min)', 0.05 / 1.6),
]
fig, ax = plt.subplots(figsize=(5.2, 1.95))
y = np.arange(len(rows))[::-1]
vals = np.array([r[1] for r in rows])
cols = [C[2] if v >= 1 else C[0] for v in vals]
ax.barh(y, vals, height=0.55, color=cols, edgecolor='white', lw=1)
for yi, v in zip(y, vals):
    ax.text(v * 1.08, yi, f'{v * 100:.0f}% of target' if v < 1 else f'{v * 100:.0f}% (met)',
            va='center', fontsize=7, color=TXT)
ax.axvline(1, color=TXT, lw=1, ls='--')
ax.set_xscale('log'); ax.set_xlim(0.02, 4)
ax.set_xticks([0.03, 0.1, 0.3, 1]); ax.set_xticklabels(['3%', '10%', '30%', '100%'])
ax.set_yticks(y); ax.set_yticklabels([r[0] for r in rows])
ax.set_xlabel('achieved as a fraction of target (log scale; ≥ 100% = met)')
ax.grid(axis='y', visible=False)
save(fig, 'kpi_status.pdf')
print('wrote link_latency.pdf, packet_delivery.pdf, drift_comparison.pdf, kpi_status.pdf')
