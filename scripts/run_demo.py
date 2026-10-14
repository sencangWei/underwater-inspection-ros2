#!/usr/bin/env python3
"""Run a finite real simulation and close all owned processes after recording."""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--output', default='evidence/run01')
parser.add_argument('--wall-timeout', type=float, default=360)
parser.add_argument('--record-window', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
output = (root / args.output).resolve()
if output.exists() and any(output.iterdir()):
    raise SystemExit('Use a new empty output directory to preserve previous runs')
output.mkdir(parents=True, exist_ok=True)
manifest = {'command': ['ros2', 'launch', 'underwater_inspection', 'inspection.launch.py'],
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'source_sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in (root / 'src').rglob('*') if p.is_file() and p.suffix in ('.py', '.yaml', '.scn')},
            'git_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()}
(output / 'run_manifest.json').write_text(json.dumps(manifest, indent=2))
recorder = None
record_log = None
with (output / 'launch.log').open('w') as log:
    process = subprocess.Popen(['ros2', 'launch', 'underwater_inspection',
        'inspection.launch.py', f'output_dir:={output}'], stdout=log,
        stderr=subprocess.STDOUT, start_new_session=True)
    started, completed = time.monotonic(), None
    try:
        while process.poll() is None:
            if args.record_window and recorder is None and (output / 'events.json').exists():
                search = subprocess.run(['xdotool', 'search', '--name', '^Stonefish Simulator$'],
                                        capture_output=True, text=True)
                if search.returncode == 0:
                    window = search.stdout.splitlines()[0]
                    subprocess.run(['xdotool', 'windowraise', window], check=True)
                    record_log = (output / 'renderer-recording.log').open('w')
                    (output / 'renderer_start.json').write_text(json.dumps({'ros_wall_time': time.time(),
                        'window_id': window, 'fps': 10, 'source': 'actual Stonefish X11 window'}))
                    recorder = subprocess.Popen(['ffmpeg', '-y', '-f', 'x11grab', '-window_id', window,
                        '-framerate', '10', '-i', os.environ.get('DISPLAY', ':0'),
                        '-an', '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast',
                        '-crf', '20', '-pix_fmt', 'yuv420p', str(output / 'renderer.mp4')],
                        stdout=record_log, stderr=subprocess.STDOUT)
            if (output / 'summary.json').exists():
                if completed is None:
                    completed = time.monotonic()
                if time.monotonic() - completed > 5:
                    break
            if time.monotonic() - started > args.wall_timeout:
                raise RuntimeError('Wall timeout; inspect launch.log')
            time.sleep(.5)
    finally:
        if recorder is not None:
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
                try:
                    recorder.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    recorder.kill()
                    recorder.wait()
            record_log.close()
        if process.poll() is None:
            # Launch forwards SIGINT to children; signalling the whole group here
            # would interrupt every child's cleanup a second time.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
summary = output / 'summary.json'
if not summary.exists():
    raise SystemExit(f'No mission result; inspect {output / "launch.log"}')
result = json.loads(summary.read_text())
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['success'] else 1)
