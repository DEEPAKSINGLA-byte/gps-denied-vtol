#!/usr/bin/env python3
"""Figures and numbers of the simulation section, from report/data/sim/ (written by
extract_sim_logs.py) and the office world file of the repository.

Usage  : python3 report/scripts/make_sim_figures.py   (from the repository root)
Outputs: report/figures/sim_*.pdf, report/data/sim/sim_stats.txt
"""
import os
import re

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.lines import Line2D

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, 'report', 'figures')
SIM = os.path.join(ROOT, 'report', 'data', 'sim')
WORLD = os.path.join(ROOT, 'ros2_ws', 'src', 'x500_navigation', 'worlds', 'office.sdf')
C = ['#257de0', '#f47230', '#00b894', '#6a4ce2']   # categorical slots 1-4 (as make_figures.py)
TXT, TXT2, GRID = '#191919', '#666666', '#e6e6e6'
WALL, TALL, LOW = '#4d4d4d', '#9a9a9a', '#e3e3e3'
STOP, PUSH = 0.65, 0.45                            # x500_px4_bridge safety distances (m)
VMAX_OFFICE = 0.5                                  # sim_office.launch.py max_speed (m/s)
MAIN = '21_45_12'                                  # complete office exploration shown in detail
SECOND = '21_53_59'
BAYLANDS = '20_20_36'

plt.rcParams.update({
    'font.family': 'serif', 'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8.5,
    'axes.edgecolor': TXT2, 'axes.labelcolor': TXT, 'xtick.color': TXT2, 'ytick.color': TXT2,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6, 'axes.spines.top': False,
    'axes.spines.right': False, 'lines.linewidth': 1.0, 'legend.frameon': False,
    'legend.fontsize': 7, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02})

runs = pd.read_csv(os.path.join(SIM, 'sim_runs.csv'), dtype={'run': str})
run = lambda r: pd.read_csv(os.path.join(SIM, f'run_{r}.csv'))
scan = lambda r: pd.read_csv(os.path.join(SIM, f'scan_{r}.csv'))
stats = []
S = lambda *a: stats.append(' '.join(str(x) for x in a))


def office_models():
    """(kind, cx, cy, sx, sy, yaw, height, z0) of every box/cylinder in office.sdf."""
    sdf = open(WORLD).read()
    out = []
    for m in re.finditer(r'<model name="([^"]+)">.*?<pose>([^<]+)</pose>.*?<geometry>(.*?)</geometry>',
                         sdf, re.S):
        name, pose, geom = m.groups()
        if name == 'ground_plane' or name.startswith('floor'):
            continue
        x, y, z, _, _, yaw = map(float, pose.split())
        b = re.search(r'<box><size>([^<]+)</size>', geom)
        if b:
            sx, sy, sz = map(float, b.group(1).split())
            out.append(('box', name, x, y, sx, sy, yaw, sz, z - sz / 2))
        c = re.search(r'<radius>([^<]+)</radius><length>([^<]+)</length>', geom)
        if c:
            r, h = float(c.group(1)), float(c.group(2))
            out.append(('cyl', name, x, y, 2 * r, 2 * r, 0.0, h, z - h / 2))
    return out


def draw_office(ax):
    """Floor plan in Gazebo coordinates (x east, y north). Walls dark, objects the 2D LiDAR
    sees at flight height (top >= 2 m) grey, low furniture (<= 1.1 m) light."""
    for kind, name, x, y, sx, sy, yaw, h, z0 in office_models():
        top = z0 + h
        if top <= 1.2:
            col, z = LOW, 1
        elif name.startswith('wall') or name.startswith('lb_entrance'):
            col, z = WALL, 3
        else:
            col, z = TALL, 2
        if kind == 'cyl':
            ax.add_patch(Circle((x, y), sx / 2, fc=col, ec='none', zorder=z))
        else:
            ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, angle=np.degrees(yaw),
                                   rotation_point='center', fc=col, ec='none', zorder=z))
    ax.set_xlim(-12.6, 12.6); ax.set_ylim(-8.6, 8.6); ax.set_aspect('equal')
    ax.set_xlabel('x, east (m)'); ax.set_ylabel('y, north (m)')
    ax.grid(False)


office = runs[runs.world == 'office']
legend_plan = [Rectangle((0, 0), 1, 1, fc=WALL), Rectangle((0, 0), 1, 1, fc=TALL),
               Rectangle((0, 0), 1, 1, fc=LOW)]
labels_plan = ['wall', 'object ≥ 2 m (in LiDAR plane)', 'furniture ≤ 1.1 m']

# ------------------------------------------------- Fig: office map, LiDAR returns, paths
fig, ax = plt.subplots(2, 1, figsize=(6.8, 9.2))
draw_office(ax[0])
MIN_HITS = 20                                      # cells hit at least this often
sc = scan(MAIN)
sc = sc[sc.hits >= MIN_HITS]
ax[0].scatter(sc.e, sc.n, s=1.6, c=C[1], lw=0, zorder=4, rasterized=True)
d = run(MAIN); f = d.offboard == 1
ax[0].plot(d.gt_e[f], d.gt_n[f], color=C[0], lw=0.9, zorder=5)
ax[0].plot(d.gt_e.iloc[0], d.gt_n.iloc[0], marker='^', ms=7, color=TXT, zorder=6, ls='none')
ax[0].set_title(f'(a) Run {MAIN.replace("_", ":")} UTC: ground-truth path and LiDAR returns',
                loc='left')
ax[0].legend(legend_plan + [Line2D([], [], color=C[0], lw=2),
                            Line2D([], [], color=C[1], marker='o', ls='none', ms=3),
                            Line2D([], [], color=TXT, marker='^', ls='none', ms=6)],
             labels_plan + ['flight path (truth)', f'LiDAR returns (10 cm cells, ≥ {MIN_HITS} hits)',
                            'take-off (lobby)'],
             loc='upper center', bbox_to_anchor=(0.5, -0.11), ncol=3, fontsize=6.5)

draw_office(ax[1])
for i, r in enumerate(office.run):
    d = run(r); f = d.offboard == 1
    row = office[office.run == r].iloc[0]
    if pd.notna(row.incident_t):
        f &= d.t <= row.incident_t
    clean = r in (MAIN, SECOND)
    ax[1].plot(d.gt_e[f], d.gt_n[f], lw=0.9 if clean else 0.6, zorder=5 if clean else 4,
               color=C[0] if r == MAIN else C[2] if r == SECOND else '#8c8c8c', alpha=1 if clean else 0.8)
inc = office[office.incident_t.notna()]
ax[1].plot(inc.incident_e, inc.incident_n, 'x', color=C[1], ms=8, mew=2, zorder=7)
groups = {}
for _, row in inc.iterrows():                      # falls at the same spot share one label
    key = (round(row.incident_e), round(row.incident_n))
    groups.setdefault(key, []).append(row)
for rows in groups.values():
    e, n = rows[0].incident_e, rows[0].incident_n
    right = e > 8
    ax[1].annotate(', '.join(r.run.replace('_', ':') for r in rows), (e, n),
                   xytext=(-6 if right else 5, 5), textcoords='offset points', fontsize=6,
                   ha='right' if right else 'left', color=TXT, zorder=8,
                   bbox=dict(fc='white', ec='none', pad=0.6, alpha=0.85))
ax[1].set_title(f'(b) All {len(office)} office flights (paths until the first fall)', loc='left')
ax[1].legend(legend_plan + [Line2D([], [], color=C[0], lw=2), Line2D([], [], color=C[2], lw=2),
                            Line2D([], [], color='#8c8c8c', lw=2),
                            Line2D([], [], color=C[1], marker='x', ls='none', ms=6, mew=2)],
             labels_plan + [f'run {MAIN.replace("_", ":")}', f'run {SECOND.replace("_", ":")}',
                            'other runs', 'start of a fall'],
             loc='upper center', bbox_to_anchor=(0.5, -0.11), ncol=4, fontsize=6.5)
fig.tight_layout(h_pad=2.5)
fig.savefig(os.path.join(FIG, 'sim_office_map.pdf'), dpi=300); plt.close(fig)

# ------------------------------------------------- Fig: time series of the main run
d = run(MAIN)
row = runs[runs.run == MAIN].iloc[0]
f = (d.offboard == 1)
fig, ax = plt.subplots(3, 1, figsize=(6.8, 5.6), sharex=True)
ax[0].plot(d.t[f], d.sp_alt[f], color=TXT2, lw=1.0, ls='--', label='commanded height')
ax[0].plot(d.t[f], d.height[f], color=C[0], label='height above floor (truth)')
ax[0].set_ylabel('height (m)'); ax[0].set_ylim(0.8, 1.9)
ax[0].set_title(f'(a) Height hold: RMS error {float(row.alt_err_rms_m)*100:.0f} cm, '
                f'largest dip {float(row.alt_err_max_m)*100:.0f} cm', loc='left')
sp = np.hypot(d.sp_vn, d.sp_ve); v = np.hypot(d.vn, d.ve)
ax[1].plot(d.t[f], sp[f], color=TXT2, lw=0.8, label='commanded (Nav2 → bridge)')
ax[1].plot(d.t[f], v[f], color=C[0], lw=0.9, label='achieved (truth)')
ax[1].axhline(VMAX_OFFICE, color=C[1], lw=1, ls=':', label='speed limit 0.5 m/s')
ax[1].set_ylabel('horizontal speed (m/s)'); ax[1].set_ylim(0, 0.75)
ax[1].set_title(f'(b) Horizontal speed: mean {float(row.speed_mean):.2f} m/s, '
                f'velocity-tracking RMS error {float(row.vel_track_rms):.2f} m/s', loc='left')
ax[2].plot(d.t[f], d.lidar_min[f], color=C[0], lw=0.8, label='nearest LiDAR return')
ax[2].axhline(STOP, color=C[1], lw=1, ls=':', label='safety: stop approach 0.65 m')
ax[2].axhline(PUSH, color=C[3], lw=1, ls='--', label='safety: push away 0.45 m')
ax[2].set_ylabel('distance (m)'); ax[2].set_ylim(0, 4)
ax[2].set_xlabel('time since OFFBOARD start (s)')
ax[2].set_title(f'(c) Clearance: minimum {float(row.lidar_min_m):.2f} m, '
                f'{float(row.frac_below_stop)*100:.1f} % of time below 0.65 m', loc='left')
for a in ax:
    a.legend(loc='upper right', ncol=3, fontsize=6.5)
fig.tight_layout(h_pad=0.6)
fig.savefig(os.path.join(FIG, 'sim_office_timeseries.pdf')); plt.close(fig)

# ------------------------------------------------- Fig: clearance CDF and speed histogram
fig, ax = plt.subplots(1, 2, figsize=(6.8, 2.7))
order = list(office.run)
for i, r in enumerate(order):
    d = run(r); row = office[office.run == r].iloc[0]
    m = (d.offboard == 1) & (d.t > 15)
    if pd.notna(row.incident_t):
        m &= d.t < row.incident_t - 2
    x = np.sort(d.lidar_min[m].dropna())
    clean = r in (MAIN, SECOND)
    ax[0].plot(x, np.arange(1, len(x) + 1) / len(x),
               color=C[0] if r == MAIN else C[2] if r == SECOND else '#a6a6a6',
               lw=1.2 if clean else 0.7, zorder=3 if clean else 2,
               label=f'run {r.replace("_", ":")}' if clean else ('other office runs' if i == 0 else None))
ax[0].axvline(STOP, color=C[1], ls=':', lw=1, label='0.65 m stop')
ax[0].axvline(PUSH, color=C[3], ls='--', lw=1, label='0.45 m push')
ax[0].set_xlim(0, 3); ax[0].set_xlabel('nearest LiDAR return (m)'); ax[0].set_ylabel('fraction of time')
ax[0].set_title('(a) Clearance distribution (office)', loc='left')
h, l = ax[0].get_legend_handles_labels()
if 'other office runs' not in l:
    h.append(Line2D([], [], color='#a6a6a6', lw=1)); l.append('other office runs')
ax[0].legend(h, l, loc='lower right', fontsize=6.3)
speeds = []
for r in (MAIN, SECOND):
    d = run(r); m = (d.offboard == 1) & (d.t > 15)
    speeds.append(np.hypot(d.vn, d.ve)[m])
ax[1].hist(speeds, bins=np.arange(0, 0.65, 0.025), color=[C[0], C[2]], rwidth=0.9,
           label=[f'run {MAIN.replace("_", ":")}', f'run {SECOND.replace("_", ":")}'])
ax[1].axvline(VMAX_OFFICE, color=C[1], ls=':', lw=1, label='limit 0.5 m/s')
ax[1].set_xlabel('horizontal speed, truth (m/s)'); ax[1].set_ylabel('samples (5 Hz)')
ax[1].set_title('(b) Speed distribution (complete explorations)', loc='left')
ax[1].legend(loc='upper left', fontsize=6.3)
fig.tight_layout(w_pad=1.5)
fig.savefig(os.path.join(FIG, 'sim_clearance_speed.pdf')); plt.close(fig)

# ------------------------------------------------- Fig: Baylands
d = run(BAYLANDS); row = runs[runs.run == BAYLANDS].iloc[0]
sc = scan(BAYLANDS); sc = sc[sc.hits >= 5]
f = d.offboard == 1
fig, ax = plt.subplots(1, 2, figsize=(6.8, 3.6), gridspec_kw=dict(width_ratios=[1, 1.35]))
ax[0].scatter(sc.e, sc.n, s=0.6, c=C[1], lw=0, rasterized=True)
ax[0].plot(d.gt_e[f], d.gt_n[f], color=C[0], lw=0.9)
ax[0].plot(d.gt_e.iloc[0], d.gt_n.iloc[0], '^', color=TXT, ms=7)
pad = 12
ax[0].set_xlim(d.gt_e[f].min() - pad, d.gt_e[f].max() + pad)
ax[0].set_ylim(d.gt_n[f].min() - pad, d.gt_n[f].max() + pad)
ax[0].set_aspect('equal'); ax[0].set_xlabel('x, east (m)'); ax[0].set_ylabel('y, north (m)')
ax[0].set_title(f'(a) Baylands, run {BAYLANDS.replace("_", ":")}', loc='left')
ax[0].legend([Line2D([], [], color=C[0], lw=2), Line2D([], [], color=C[1], marker='o', ls='none', ms=3),
              Line2D([], [], color=TXT, marker='^', ls='none', ms=6)],
             ['path (truth)', 'LiDAR returns (≥ 5 hits)', 'take-off'], loc='upper center',
             bbox_to_anchor=(0.5, -0.16), ncol=2, fontsize=6.3)
ax[1].plot(d.t[f], d.est_err[f] * 100, color=C[0], lw=0.8)
ax[1].set_xlabel('time since OFFBOARD start (s)'); ax[1].set_ylabel('|estimate − truth| (cm)')
ax[1].set_title('(b) EKF2 horizontal position error (GPS-aided)', loc='left')
fig.tight_layout(w_pad=1.5)
fig.savefig(os.path.join(FIG, 'sim_baylands.pdf'), dpi=300); plt.close(fig)

# ------------------------------------------------- numbers quoted in the text
for _, r in runs.iterrows():
    S(*[f'{k}={r[k]}' for k in runs.columns])
fly = runs[runs.path_m.astype(float) > 5]
S('total_runs', len(runs), 'office', len(office), 'baylands', (runs.world == 'baylands').sum())
S('total_offboard_s', runs.offboard_s.sum(), 'total_path_m_pre_incident', round(runs.path_m.sum(), 1))
S('runs_moving_gt5m', len(fly), 'est_err_rms_max_over_runs', fly.est_err_rms_m.max(),
  'est_err_max_over_runs', fly.est_err_max_m.max())
S('falls_office', office.incident_t.notna().sum())
open(os.path.join(SIM, 'sim_stats.txt'), 'w').write('\n'.join(stats) + '\n')
print('\n'.join(stats[-5:]))


# ------------------------------------------------- LaTeX tables (\input by 07_validation.tex)
# Whole tabular environments: \input cannot sit between \midrule and \bottomrule.
RUNS_HEAD = r"""\begin{tabular}{@{}l l r r r r r r r r r r l@{}}
\toprule
& & \multicolumn{1}{c}{\textbf{OFFB.}} & \multicolumn{1}{c}{\textbf{Path}} &
\multicolumn{2}{c}{\textbf{Height (cm)}} & \multicolumn{2}{c}{\textbf{Speed (m/s)}} &
\multicolumn{1}{c}{\textbf{Track}} & \multicolumn{1}{c}{\textbf{Clear.}} &
\multicolumn{1}{c}{\textbf{$<$0.65}} & \multicolumn{1}{c}{\textbf{Est.}} & \\
\cmidrule(lr){5-6}\cmidrule(lr){7-8}
\textbf{Run (UTC)} & \textbf{World} & (s) & (m) & RMS & max & mean & p95 & (m/s) &
min (m) & (\%) & (cm) & \textbf{Outcome} \\
\midrule
"""
INC_HEAD = r"""\begin{tabular}{@{}l r l l r r l@{}}
\toprule
\textbf{Run (UTC)} & \textbf{Time (s)} & \textbf{Position (m)} & \textbf{Nearest object} &
\textbf{Dist.\ (m)} & \textbf{LiDAR (m)} & \textbf{Afterwards} \\
\midrule
"""
TAIL = '\\bottomrule\n\\end{tabular}\n'
GEN = '% generated by report/scripts/make_sim_figures.py from sim_runs.csv - do not edit\n'
def nearest_object(e, n):
    """Closest object of office.sdf to the point (east e, north n): (distance, name, top)."""
    best = (np.inf, '', 0.0)
    for kind, name, x, y, sx, sy, yaw, h, z0 in office_models():
        if abs(np.sin(yaw)) > 0.5:
            sx, sy = sy, sx
        dist = (max(np.hypot(e - x, n - y) - sx / 2, 0) if kind == 'cyl' else
                np.hypot(max(abs(e - x) - sx / 2, 0), max(abs(n - y) - sy / 2, 0)))
        if dist < best[0]:
            best = (dist, name, z0 + h)
    return best


def cell(v, scale=1.0, p=2):
    return '--' if pd.isna(v) or v == '' else f'{float(v) * scale:.{p}f}'


lines = []
show = runs[(runs.world == 'office') | (runs.path_m.astype(float) > 10)]
for _, r in show.iterrows():
    outcome = []
    if isinstance(r.explore_end, str) and r.explore_end:
        outcome.append({'all frontiers traversed/tried': 'explored, all frontiers',
                        'no frontiers left': 'explored, no frontiers',
                        'stopped by operator': 'exploring, stopped'}[r.explore_end])
    if pd.notna(r.incident_t):
        outcome.append(f'fall at {float(r.incident_t):.0f}\\,s')
    world = 'Office' if r.world == 'office' else ('Baylands*' if r.spawn == 'baylands-origin' else 'Baylands')
    lines.append(' & '.join([
        r.run.replace('_', ':'), world, f'{int(r.offboard_s)}', cell(r.path_m, p=1),
        cell(r.alt_err_rms_m, 100, 0), cell(r.alt_err_max_m, 100, 0),
        cell(r.speed_mean), cell(r.speed_p95), cell(r.vel_track_rms),
        cell(r.lidar_min_m), cell(r.frac_below_stop, 100, 1), cell(r.est_err_rms_m, 100, 1),
        '; '.join(outcome) or 'goals by operator']) + r' \\')
open(os.path.join(SIM, 'sim_runs_table.tex'), 'w').write(
    GEN + RUNS_HEAD + '\n'.join(lines) + '\n' + TAIL)

def after(r):
    """What the logs show after a fall: did it climb back, how did the run end."""
    d = run(r.run)
    t = float(r.incident_t)
    back = d[(d.t > t + 5) & (d.t < t + 40) & (d.offboard == 1)].height.max() > 1.3
    S('incident', r.run, 'climbed_back_above_1.3m_within_40s', back)
    end = {'all frontiers traversed/tried': 'exploration completed',
           'no frontiers left': 'no frontiers left',
           'stopped by operator': 'stopped by operator'}.get(r.explore_end, 'goals by operator')
    return ('climbed back; ' if back else 'did not climb back; ') + end


lines = []
for _, r in office[office.incident_t.notna()].iterrows():
    dist, name, top = nearest_object(float(r.incident_e), float(r.incident_n))
    S('incident', r.run, 'nearest', name, 'dist_m', round(dist, 2), 'top_m', round(top, 2))
    kind = ('wall' if name.startswith('wall') else 'plant' if 'plant' in name else
            'chair' if 'chair' in name else name)
    lines.append(' & '.join([
        r.run.replace('_', ':'), f'{float(r.incident_t):.1f}',
        f'({float(r.incident_e):.2f}, {float(r.incident_n):.2f})',
        f'{kind}, top {top:.2f}\\,m', f'{dist:.2f}', cell(r.incident_lidar_min_m),
        after(r)]) + r' \\')
open(os.path.join(SIM, 'sim_incidents_table.tex'), 'w').write(
    GEN + INC_HEAD + '\n'.join(lines) + '\n' + TAIL)
open(os.path.join(SIM, 'sim_stats.txt'), 'w').write('\n'.join(stats) + '\n')
