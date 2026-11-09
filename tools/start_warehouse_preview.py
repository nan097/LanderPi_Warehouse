#!/usr/bin/env python3
"""Keep the warehouse GUI alive, with a dedicated ROS domain and log."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
pidfile=Path('/tmp/landerpi_warehouse_preview.pid')
if pidfile.exists():
 try:
  pid=int(pidfile.read_text());args=Path(f'/proc/{pid}/cmdline').read_bytes()
  if b'warehouse_simulation.launch.py' in args and b'ros_domain_id:=79' in args:
   print('Warehouse preview already running',pid);raise SystemExit(0)
 except (OSError,ValueError):pass
with Path('/tmp/landerpi_warehouse_visible.log').open('w') as log:
 p=subprocess.Popen(['bash','tools/run_warehouse_simulation.sh','ros_domain_id:=79','autorun:=false'],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
pidfile.write_text(str(p.pid)+'\n')
print('Warehouse preview PID:',p.pid,'Log: /tmp/landerpi_warehouse_visible.log')
