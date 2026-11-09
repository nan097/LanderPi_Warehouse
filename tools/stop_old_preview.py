import os,signal
from pathlib import Path
pid=19403
p=Path(f'/proc/{pid}/cmdline')
if p.exists() and b'landerpi_bringup' in p.read_bytes() and b'demo:=true' in p.read_bytes():
 os.kill(pid,signal.SIGINT)
 print('Stopped earlier preview launch',pid)
