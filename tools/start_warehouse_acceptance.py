#!/usr/bin/env python3
"""Start the long acceptance in its own session and retain its output."""
import json
from pathlib import Path
import subprocess
import time
ROOT=Path(__file__).resolve().parents[1]
output=Path(f'/tmp/warehouse_acceptance_stdout_{time.time_ns()}.txt')
with output.open('w') as stream:
    process=subprocess.Popen(['bash','tools/run_warehouse_test.sh'],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
state={'pid':process.pid,'stdout':str(output),'summary':f'/tmp/landerpi_warehouse_acceptance_{process.pid}.json','log':f'/tmp/landerpi_warehouse_acceptance_{process.pid}.log'}
Path('/tmp/warehouse_acceptance_active.json').write_text(json.dumps(state,indent=2))
print(json.dumps(state,indent=2))
