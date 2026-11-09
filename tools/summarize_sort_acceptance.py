#!/usr/bin/env python3
"""Summarize measured grasp geometry and task CSV; never infer a passing test."""
import argparse
import csv
import gzip
import json
import math
import re
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('log', type=Path)
parser.add_argument('csv', type=Path)
parser.add_argument('--output', type=Path)
args = parser.parse_args()
pattern = r'x=([-+0-9.eE]+), y=([-+0-9.eE]+), z=([-+0-9.eE]+)'
detection = None
detection_errors = []
contact_errors = []
log_text = (gzip.open(args.log, 'rt', errors='replace').read()
            if args.log.suffix == '.gz' else args.log.read_text(errors='replace'))
for line in log_text.splitlines():
    points = [tuple(map(float, values)) for values in re.findall(pattern, line)]
    if 'RGB-D centroid' in line and points:
        detection = points[0]
    if 'Grasp verification before close' in line and len(points) == 2:
        contact_errors.append(math.dist(points[0], points[1]))
        if detection is not None:
            detection_errors.append(math.dist(detection, points[0]))
with args.csv.open(newline='') as stream:
    rows = list(csv.DictReader(stream))
success = [row for row in rows if row['success'].lower() == 'true']
summary = {
    'task_records': len(rows),
    'successful_tasks': len(success),
    'task_duration_s': [float(row['duration_s']) for row in success],
    'measured_grasps': len(contact_errors),
    'max_rgbd_position_error_m': max(detection_errors, default=None),
    'max_tool_object_distance_m': max(contact_errors, default=None),
    'errors': [{'task_id': row['task_id'], 'error_code': int(row['error_code'])}
               for row in rows if row['success'].lower() != 'true'],
    'source_log': str(args.log),
    'source_csv': str(args.csv),
}
text = json.dumps(summary, indent=2, ensure_ascii=False)
if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text + '\n')
print(text)
