#!/usr/bin/env python3
"""Stop only the warehouse preview created by start_warehouse_preview.py."""
from pathlib import Path
import os,signal
pidfile=Path('/tmp/landerpi_warehouse_preview.pid')
if not pidfile.exists():print('No recorded warehouse preview');raise SystemExit(0)
pid=int(pidfile.read_text())
try:args=Path(f'/proc/{pid}/cmdline').read_bytes()
except FileNotFoundError:print('Warehouse preview already stopped');raise SystemExit(0)
if b'warehouse_simulation.launch.py' not in args or b'ros_domain_id:=79' not in args:raise RuntimeError('Recorded process is not the warehouse preview')
os.kill(pid,signal.SIGINT);print('Stopping warehouse preview',pid)
