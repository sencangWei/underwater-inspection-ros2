#!/usr/bin/env python3
"""Plot recorded trajectory and visual range; all points come from telemetry."""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('run', type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
cfg = yaml.safe_load((root / 'src/underwater_inspection/config/mission.yaml').read_text())
rows = list(csv.DictReader((args.run / 'telemetry.csv').open()))
start = float(rows[0]['ros_wall_time'])
fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
ax = axes[0]
for obstacle in cfg['obstacles']:
    ax.add_patch(Circle((obstacle['x'], obstacle['y']), obstacle['radius'], color='#64748b'))
    ax.add_patch(Circle((obstacle['x'], obstacle['y']), obstacle['radius'] + cfg['robot_radius'],
                        fill=False, color='#94a3b8', linestyle='--'))
for phase, color in [('TRANSIT_AVOID_OBSTACLES', '#0284c7'), ('VISUAL_APPROACH', '#f59e0b'),
                      ('INSPECT_HOLD', '#16a34a'), ('RETURN_HOME', '#7c3aed')]:
    points = [r for r in rows if r['phase'] == phase]
    if points:
        ax.plot([float(r['x']) for r in points], [float(r['y']) for r in points],
                color=color, label=phase, linewidth=2)
ax.scatter(*cfg['spawn'][:2], marker='s', color='black', label='home')
ax.scatter(*cfg['target'][:2], marker='*', s=110, color='#f59e0b', label='panel')
ax.set(xlabel='NED x [m]', ylabel='NED y [m]', title='Actual simulated odometry trajectory',
       xlim=(-6, 6), ylim=(-4, 4))
ax.set_aspect('equal')
ax.legend(fontsize=7)
ax.grid(alpha=.2)
range_rows = [r for r in rows if r['panel_range'] and r['phase'] in ('VISUAL_APPROACH', 'INSPECT_HOLD')]
axes[1].plot([float(r['ros_wall_time']) - start for r in range_rows],
             [float(r['panel_range']) for r in range_rows], label='camera geometric range', color='#0284c7')
axes[1].axhline(cfg['standoff'], linestyle='--', color='#16a34a', label='desired standoff')
axes[1].set(xlabel='Elapsed wall time [s]', ylabel='Camera range [m]', title='Vision-guided approach and inspection')
axes[1].legend(fontsize=8)
axes[1].grid(alpha=.2)
fig.tight_layout()
fig.savefig(args.run / 'trajectory-and-range.png', dpi=160)
