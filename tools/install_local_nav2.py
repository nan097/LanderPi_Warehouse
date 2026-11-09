#!/usr/bin/env python3
"""Install missing Nav2 dependency files inside this workspace, without sudo."""
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1] / '.deps'
cache = root / 'downloads'
cache.mkdir(parents=True, exist_ok=True)
packages = ['ros-humble-nav2-' + p for p in ['bt-navigator', 'controller', 'dwb-controller', 'planner', 'navfn-planner', 'behaviors', 'map-server', 'lifecycle-manager']]
packages.append('ros-humble-rmw-cyclonedds-cpp')
plan = subprocess.check_output(['apt-get', '-s', 'install', *packages], text=True)
missing = [line.split()[1] for line in plan.splitlines() if line.startswith('Inst ')]
if missing:
    subprocess.run(['apt-get', 'download', *missing], cwd=cache, check=True)
for deb in cache.glob('*.deb'):
    subprocess.run(['dpkg-deb', '-x', str(deb), str(root / 'nav2_root')], check=True)
subprocess.run(['python3', str(Path(__file__).with_name('relocate_local_nav2.py'))], check=True)
print('Local Nav2 dependencies extracted:', len(missing))
