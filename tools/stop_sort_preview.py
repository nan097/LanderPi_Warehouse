"""Stop only the GUI process recorded by start_sort_preview.py."""
from pathlib import Path
import os
import signal

pid_file = Path('/tmp/landerpi_sort_preview.pid')
if not pid_file.exists():
    print('No recorded preview')
    raise SystemExit(0)
pid = int(pid_file.read_text().strip())
try:
    args = Path(f'/proc/{pid}/cmdline').read_bytes()
except FileNotFoundError:
    print('Preview already stopped')
    raise SystemExit(0)
if b'ros_domain_id:=76' not in args or not (b'run_sort_demo.sh' in args or b'sort_simulation.launch.py' in args):
    raise RuntimeError('Recorded PID no longer belongs to this preview; refusing to signal it')
os.kill(pid, signal.SIGINT)
print('Stopping preview', pid)
