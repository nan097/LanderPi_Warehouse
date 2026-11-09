#!/usr/bin/env python3
"""Download ROS Humble MTC debs into the existing local dependency prefix."""
from pathlib import Path
import subprocess

workspace = Path(__file__).resolve().parents[1]
cache = workspace/'.deps/mtc_debs'
cache.mkdir(parents=True, exist_ok=True)
packages = ['ros-humble-moveit-task-constructor-core',
            'ros-humble-moveit-task-constructor-msgs',
            'ros-humble-moveit-task-constructor-capabilities',
            'ros-humble-rviz-marker-tools', 'ros-humble-py-binding-tools']
subprocess.run(['apt-get', 'download', *packages], cwd=cache, check=True)
for deb in sorted(cache.glob('*.deb')):
    subprocess.run(['dpkg-deb', '-x', str(deb), str(workspace/'.deps/nav2_root')], check=True)
print('MTC extracted. Source tools/simulation_env.sh before building/running.')
