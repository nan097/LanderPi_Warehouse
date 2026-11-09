"""Keep the verified GUI demo alive independently of the command session."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
log = Path('/tmp/landerpi_sort_visible.log')
pid_file = Path('/tmp/landerpi_sort_preview.pid')
if pid_file.exists():
    try:
        previous_pid = int(pid_file.read_text().strip())
        args = Path(f'/proc/{previous_pid}/cmdline').read_bytes()
        if b'ros_domain_id:=76' in args and (b'run_sort_demo.sh' in args or b'sort_simulation.launch.py' in args):
            print('Preview already running:', previous_pid, 'Log:', log)
            raise SystemExit(0)
    except (OSError, ValueError):
        pass
with log.open('w') as stream:
    process = subprocess.Popen(['bash', 'tools/run_sort_demo.sh', 'ros_domain_id:=76'],
                               cwd=root, stdin=subprocess.DEVNULL, stdout=stream,
                               stderr=subprocess.STDOUT, start_new_session=True)
pid_file.write_text(str(process.pid)+'\n')
print('Preview PID:', process.pid, 'ROS domain: 76', 'Log:', log)
