"""Stop only the isolated acceptance launch identified by its numeric runner ID."""
from pathlib import Path
import os,signal,sys
run=int(sys.argv[1])
expected=f'csv_path:=/tmp/landerpi_warehouse_acceptance_{run}.csv'.encode()
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():continue
    try:args=(entry/'cmdline').read_bytes().split(b'\0')
    except OSError:continue
    if b'launch' in args and b'warehouse_simulation.launch.py' in args and expected in args:
        try:os.kill(int(entry.name),signal.SIGINT);print('Stopped own warehouse acceptance launch',entry.name)
        except ProcessLookupError:pass
