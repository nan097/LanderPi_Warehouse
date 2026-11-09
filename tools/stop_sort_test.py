"""Interrupt only our named integration runner; its finally block stops its launch."""
from pathlib import Path
import os,signal
for entry in Path('/proc').iterdir():
 if entry.name.isdigit():
  try: args=(entry/'cmdline').read_bytes().split(b'\0')
  except OSError:continue
  if len(args)>1 and args[1].endswith(b'tools/test_sort_simulation.py'):
   os.kill(int(entry.name),signal.SIGINT);print('Interrupted integration test',entry.name)

# Also stop only launches carrying our uniquely named acceptance CSV argument.
for entry in Path('/proc').iterdir():
 if not entry.name.isdigit():continue
 try:args=(entry/'cmdline').read_bytes().split(b'\0')
 except OSError:continue
 if b'launch' in args and b'sort_simulation.launch.py' in args and any(arg.startswith(b'csv_path:=/tmp/landerpi_sort_acceptance_') for arg in args):
  try:os.kill(int(entry.name),signal.SIGINT);print('Stopped acceptance launch',entry.name)
  except ProcessLookupError:pass
