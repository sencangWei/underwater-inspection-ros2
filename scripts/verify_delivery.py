#!/usr/bin/env python3
"""Validate recorded mission, source provenance, video and review artefacts."""
import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('run', type=Path)
parser.add_argument('--video', type=Path, default=Path('demo/underwater-inspection-demo.mp4'))
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
result = json.loads((args.run / 'summary.json').read_text())
assert result['success'] and not result['contact_bodies']
assert result['current']['success']
assert result['inspection_completed_seconds'] >= result['inspection_required_seconds']
assert result['minimum_obstacle_clearance_m'] > .2
assert result['home_error_m'] < .25
phases = [event['phase'] for event in result['events']]
assert phases == ['TRANSIT_AVOID_OBSTACLES', 'SEARCH_PANEL', 'VISUAL_APPROACH',
                  'INSPECT_HOLD', 'RETURN_HOME', 'COMPLETE']
for name in ('frame_times.csv', 'telemetry.csv'):
    stamps = [float(r['ros_wall_time']) for r in csv.DictReader((args.run / name).open())]
    assert all(b > a for a, b in zip(stamps, stamps[1:])), name
manifest = json.loads((args.run / 'run_manifest.json').read_text())
for name, digest in manifest['source_sha256'].items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == digest, name
video = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
    '-show_entries', 'stream=codec_name,width,height,r_frame_rate,nb_frames:format=duration,size',
    '-of', 'json', str(args.video)], text=True))
stream = video['streams'][0]
alignment = json.loads(args.video.with_suffix('.json').read_text())
assert stream['codec_name'] == 'h264' and stream['width'] == 1920 and stream['height'] == 1080
assert stream['r_frame_rate'] == '10/1'
assert int(stream['nb_frames']) == alignment['output_frames']
assert abs(float(video['format']['duration']) - alignment['output_duration_s']) < .01
assert alignment['end_wall_time'] > result['events'][-1]['ros_wall_time'] + 2
assert (args.run / 'inspection-photo.png').exists()
assert len(list((args.run / 'screenshots').glob('*.png'))) >= 6
print(json.dumps({'status': 'PASS', 'run': str(args.run),
    'mission_seconds': result['duration_wall_seconds'],
    'minimum_clearance_m': result['minimum_obstacle_clearance_m'],
    'camera_range_rmse_m': result['inspection_range_rmse_m'],
    'home_error_m': result['home_error_m'], 'sensor_pause_recoveries': result['sensor_pause_count'],
    'verified_source_files': len(manifest['source_sha256']),
    'video_seconds': float(video['format']['duration']), 'video_bytes': int(video['format']['size'])}, indent=2))
