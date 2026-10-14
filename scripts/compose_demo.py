#!/usr/bin/env python3
"""Synchronize actual renderer and sensor recordings using recorded wall times."""
import argparse
import bisect
import csv
import json
import math
import re
import subprocess
from pathlib import Path

import cv2
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('run', type=Path)
parser.add_argument('--output', type=Path, default=Path('demo/underwater-inspection-demo.mp4'))
args = parser.parse_args()
run = args.run.resolve()
times = [float(row['ros_wall_time']) for row in csv.DictReader((run / 'frame_times.csv').open())]
rows = list(csv.DictReader((run / 'telemetry.csv').open()))
row_times = [float(row['ros_wall_time']) for row in rows]
summary = json.loads((run / 'summary.json').read_text())
renderer_start = json.loads((run / 'renderer_start.json').read_text())['ros_wall_time']
log = (run / 'renderer-recording.log').read_text(errors='replace')
match = re.search(r'Duration: N/A, start: ([0-9.]+)', log)
if match:
    renderer_start = float(match.group(1))
sensors = cv2.VideoCapture(str(run / 'sensors-telemetry.mp4'))
renderer = cv2.VideoCapture(str(run / 'renderer.mp4'))
sensor_count = round(sensors.get(cv2.CAP_PROP_FRAME_COUNT))
renderer_count = round(renderer.get(cv2.CAP_PROP_FRAME_COUNT))
renderer_fps = renderer.get(cv2.CAP_PROP_FPS)
if sensor_count != len(times) or not times or renderer_count < 10 or renderer_fps < 1:
    raise SystemExit('Source videos or timestamp records are incomplete')
start = max(times[0], renderer_start)
end = min(times[-1], renderer_start + (renderer_count - 1) / renderer_fps)
if end - start < 10:
    raise SystemExit('Too little synchronized recording')
root = Path(__file__).resolve().parents[1]
raw = root / '.local/synchronized-demo-raw.mp4'
raw.parent.mkdir(exist_ok=True)
args.output.parent.mkdir(parents=True, exist_ok=True)
fps = 10
writer = cv2.VideoWriter(str(raw), cv2.VideoWriter_fourcc(*'mp4v'), fps, (1920, 1080))
if not writer.isOpened():
    raise SystemExit('Cannot create synchronized video')
sensor_index = renderer_index = -1
sensor_frame = renderer_frame = None
frame_count = math.floor((end - start) * fps) + 1

for i in range(frame_count):
    stamp = start + i / fps
    target_sensor = min(sensor_count - 1, max(0, bisect.bisect_right(times, stamp) - 1))
    while sensor_index < target_sensor:
        ok, sensor_frame = sensors.read()
        if not ok:
            raise RuntimeError('Sensor frame decode failed')
        sensor_index += 1
    target_renderer = min(renderer_count - 1, max(0, round((stamp - renderer_start) * renderer_fps)))
    while renderer_index < target_renderer:
        ok, renderer_frame = renderer.read()
        if not ok:
            raise RuntimeError('Renderer frame decode failed')
        renderer_index += 1
    row = rows[max(0, bisect.bisect_right(row_times, stamp) - 1)]
    canvas = np.full((1080, 1920, 3), (23, 28, 35), np.uint8)
    cv2.putText(canvas, 'UNDERWATER AUTONOMOUS INSPECTION', (38, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 1.25, (238, 246, 250), 2, cv2.LINE_AA)
    cv2.putText(canvas, 'ROS 2 Humble + Stonefish | actual simulation, camera and sonar recordings',
                (40, 100), cv2.FONT_HERSHEY_SIMPLEX, .68, (150, 195, 205), 1, cv2.LINE_AA)
    cv2.putText(canvas, '3D PHYSICS | BlueROV2 with ocean current', (20, 143),
                cv2.FONT_HERSHEY_SIMPLEX, .63, (222, 233, 240), 1, cv2.LINE_AA)
    cv2.putText(canvas, 'RGB VISUAL SERVO | FLS + live trajectory', (985, 143),
                cv2.FONT_HERSHEY_SIMPLEX, .63, (222, 233, 240), 1, cv2.LINE_AA)
    canvas[165:885, :960] = cv2.resize(renderer_frame, (960, 720))
    canvas[200:740, 960:] = cv2.resize(sensor_frame, (960, 540))
    age = stamp - times[target_sensor]
    if age > .5:
        cv2.rectangle(canvas, (970, 750), (1900, 794), (25, 55, 95), -1)
        cv2.putText(canvas, f'SENSOR FRAME DELAY {age:.1f}s | holding previous measured frame',
                    (984, 778), cv2.FONT_HERSHEY_SIMPLEX, .61, (160, 215, 255), 1, cv2.LINE_AA)
    panel_range = row['panel_range'] or '--'
    lines = [f"PHASE: {row['phase']}",
             f"pose NED=({float(row['x']):.2f}, {float(row['y']):.2f}, {float(row['z']):.2f}) m",
             f"panel={panel_range[:5]} m | hold={float(row['inspect_good_seconds']):.1f}/8.0 s"]
    for j, line in enumerate(lines):
        cv2.putText(canvas, line, (984, 830 + j * 38), cv2.FONT_HERSHEY_SIMPLEX,
                    .68, (222, 239, 233), 1, cv2.LINE_AA)
    cv2.putText(canvas, f't={stamp - times[0]:.1f}s | CURRENT 0.10 m/s | '+
                f"contacts={len(summary['contact_bodies'])}", (35, 956),
                cv2.FONT_HERSHEY_SIMPLEX, .75, (180, 228, 220), 1, cv2.LINE_AA)
    cv2.putText(canvas, 'Baseline: known-site planning + simulator odometry + geometric marker perception',
                (35, 1013), cv2.FONT_HERSHEY_SIMPLEX, .63, (155, 165, 178), 1, cv2.LINE_AA)
    writer.write(canvas)

writer.release()
sensors.release()
renderer.release()
subprocess.run(['ffmpeg', '-y', '-v', 'error', '-i', str(raw), '-an', '-c:v', 'libx264',
                '-preset', 'medium', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                str(args.output)], check=True)
manifest = {'run': str(run.relative_to(root)), 'start_wall_time': start, 'end_wall_time': end,
            'output_frames': frame_count, 'output_fps': fps, 'output_duration_s': frame_count / fps,
            'sensor_source_frames': sensor_count, 'sensor_actual_span_s': times[-1] - times[0],
            'sensor_mean_hz': (sensor_count - 1) / (times[-1] - times[0]),
            'renderer_start_wall_time': renderer_start,
            'sensor_gaps_over_half_second': sum(b - a > .5 for a, b in zip(times, times[1:])),
            'alignment': 'actual X11 start timestamp + actual sensor frame wall timestamps; repeat previous observation during gaps'}
args.output.with_suffix('.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps(manifest, indent=2))
