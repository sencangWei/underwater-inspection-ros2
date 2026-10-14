#!/usr/bin/env python3
"""Run a finite real simulation and close all owned processes after recording."""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--output', default='evidence/run01')
parser.add_argument('--wall-timeout', type=float, default=360)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
output = (root / args.output).resolve()
output.mkdir(parents=True, exist_ok=True)
if (output / 'summary.json').exists():
    raise SystemExit('Use a new output directory to preserve previous runs')
with (output / 'launch.log').open('w') as log:
    process = subprocess.Popen(['ros2', 'launch', 'underwater_inspection',
        'inspection.launch.py', f'output_dir:={output}'], stdout=log,
        stderr=subprocess.STDOUT, start_new_session=True)
    started, completed = time.monotonic(), None
    try:
        while process.poll() is None:
            if (output / 'summary.json').exists():
                if completed is None:
                    completed = time.monotonic()
                if time.monotonic() - completed > 5:
                    break
            if time.monotonic() - started > args.wall_timeout:
                raise RuntimeError('Wall timeout; inspect launch.log')
            time.sleep(.5)
    finally:
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
