#!/usr/bin/env python3
"""Regression probe: an unrelated legacy /scene_info provider on another master."""
from pathlib import Path
import json,os,socket,subprocess
ROOT=Path(__file__).resolve().parents[1]
world=Path('/tmp/landerpi_legacy_scene_probe.world')
world.write_text('''<sdf version="1.6"><world name="legacy_scene_probe"><model name="LEGACY_ONLY"><static>true</static><link name="box"><visual name="box"><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>''')
with socket.socket() as probe:probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
env=os.environ.copy();env['GAZEBO_MASTER_URI']=f'http://127.0.0.1:{port}';env.pop('IGN_PARTITION',None)
with Path('/tmp/landerpi_legacy_scene_probe.log').open('w') as log:
 p=subprocess.Popen(['gzserver','--verbose',str(world)],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
state=dict(pid=p.pid,master=env['GAZEBO_MASTER_URI'])
Path('/tmp/landerpi_legacy_scene_probe.json').write_text(json.dumps(state));print(json.dumps(state))
