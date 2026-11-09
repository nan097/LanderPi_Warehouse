#!/usr/bin/env python3
"""Verify scene-service discovery isolation; inspect the saved GUI image for models."""
import argparse,json,os,re,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--partition',required=True);p.add_argument('--compare-legacy',action='store_true');p.add_argument('--output',default='/tmp/landerpi_gazebo_scene_check.json');args=p.parse_args()
def providers(partition):
 env=os.environ.copy()
 if partition:env['IGN_PARTITION']=partition
 else:env.pop('IGN_PARTITION',None)
 result=subprocess.run(['ign','service','-i','-s','/scene_info'],env=env,text=True,capture_output=True,timeout=20)
 addresses=re.findall(r'tcp://[^,\s]+',result.stdout)
 if result.returncode or not addresses:raise RuntimeError('Scene provider unavailable: '+result.stdout+result.stderr)
 return addresses
current=providers(args.partition)
assert len(current)==1,('ambiguous warehouse scene providers',current)
report=dict(success=True,partition=args.partition,scene_info_providers=current)
if args.compare_legacy:
 legacy=providers(None)
 assert set(current).isdisjoint(legacy),'legacy scene provider visible in warehouse partition'
 report.update(legacy_scene_info_providers=legacy,isolated_from_legacy=True)
Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
