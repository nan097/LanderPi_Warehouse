#!/usr/bin/env python3
"""Clean up only the legacy scene probe created by start_gazebo_scene_probe.py."""
import json,os,signal
from pathlib import Path
state=Path('/tmp/landerpi_legacy_scene_probe.json')
if not state.exists():print('No recorded scene probe');raise SystemExit(0)
pid=json.loads(state.read_text())['pid']
try:args=Path(f'/proc/{pid}/cmdline').read_bytes()
except FileNotFoundError:print('Scene probe already stopped');raise SystemExit(0)
if b'gzserver' not in args or b'/tmp/landerpi_legacy_scene_probe.world' not in args:raise RuntimeError('Recorded PID belongs to a different process')
os.kill(pid,signal.SIGINT);print('Stopping scene probe',pid)
