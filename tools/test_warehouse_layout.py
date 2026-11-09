"""Verify inventory capacity and geometry independently of robot execution."""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import yaml
import cv2
import numpy as np
from collections import deque
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/landerpi_simulation/scripts'))
from warehouse_layout import validate,fixture_parts,slot,inside,dock
SHARE=ROOT/'src/landerpi_simulation'

class WarehouseGeometry(unittest.TestCase):
    def setUp(self):self.config=yaml.safe_load((SHARE/'config/warehouse.yaml').read_text())
    def test_inventory_and_capacity(self):
        goods=validate(self.config);self.assertEqual(len(goods),5)
        self.assertEqual(len({g['rack'] for g in goods}),5)
        self.assertEqual(len(self.config['racks']),6);self.assertEqual(len(self.config['outputs']),3)
        self.assertEqual({c:sum(g['color']==c for g in goods) for c in self.config['colors']},{'red':2,'green':2,'blue':1})
    def test_compact_ground_level_environment(self):
        xmin,xmax,ymin,ymax=self.config['room']
        self.assertAlmostEqual(xmax-xmin,2.4);self.assertAlmostEqual(ymax-ymin,2.0)
        parts=fixture_parts(self.config)
        for name in [*self.config['racks'],*self.config['outputs']]:
            body=[(p,size) for n,p,size in parts if n.startswith(name+'_')]
            self.assertEqual(len(body),5,'ground boxes must not have stands or posts')
            self.assertAlmostEqual(min(p[2]-size[2]/2 for p,size in body),0.)
        for name,p,size in parts:
            if name.startswith('wall_'):self.assertAlmostEqual(p[2]-size[2]/2,0.)

    def test_grasp_has_position_error_margin(self):
        offset=self.config['container_inner'][2]+self.config['grasp_tool_clearance']-self.config['box_size'][2]/2
        self.assertLessEqual(offset,.027,'reserve at least 3 mm below the 30 mm grasp limit')

    def test_exact_container_envelope_and_cavity(self):
        parts=fixture_parts(self.config)
        for name in [*self.config['racks'],*self.config['outputs']]:
            box=[(p,size) for n,p,size in parts if n==name+'_support' or n.startswith(name+'_end_') or n.startswith(name+'_side_')]
            self.assertEqual(len(box),5)
            for axis,length in enumerate([.119,.0857,.0538]):
                lower=min(p[axis]-size[axis]/2 for p,size in box)
                upper=max(p[axis]+size[axis]/2 for p,size in box)
                self.assertAlmostEqual(upper-lower,length)
            ends=[(p,size) for n,p,size in parts if n.startswith(name+'_end_')]
            sides=[(p,size) for n,p,size in parts if n.startswith(name+'_side_')]
            self.assertAlmostEqual(ends[1][0][0]-ends[1][1][0]/2-(ends[0][0][0]+ends[0][1][0]/2),.10)
            self.assertAlmostEqual(sides[1][0][1]-sides[1][1][1]/2-(sides[0][0][1]+sides[0][1][1]/2),.08)
            self.assertAlmostEqual(ends[0][1][2],.05)
    def test_physical_world_matches_planning_geometry(self):
        world=ET.parse(SHARE/'worlds/warehouse.world').getroot().find('world')
        collisions={c.get('name'):c for c in world.findall('model/link/collision')}
        for name,position,size in fixture_parts(self.config):
            self.assertIn(name,collisions)
            self.assertEqual(list(map(float,collisions[name].find('pose').text.split()[:3])),position)
            self.assertEqual(list(map(float,collisions[name].find('geometry/box/size').text.split())),size)
        for item in validate(self.config):
            model=world.find(f"model[@name='{item['id']}']")
            self.assertIsNone(model.find('static'))
            self.assertGreater(float(model.find('link/inertial/mass').text),0)
            self.assertAlmostEqual(float(model.find('pose').text.split()[2]),self.config['support_height']+self.config['box_size'][2]/2)
    def test_camera_is_on_robot_and_not_in_world(self):
        world=ET.parse(SHARE/'worlds/warehouse.world').getroot()
        self.assertEqual(world.findall('.//sensor'),[])
        self.assertFalse(any('camera_' in name for name,_,_ in fixture_parts(self.config)))
        robot=ET.parse(SHARE/'urdf/landerpi_sim.urdf.xacro').getroot()
        sensor=robot.find(".//gazebo[@reference='depth_cam_link']/sensor")
        self.assertEqual(sensor.get('type'),'depth')
        joint=robot.find(".//joint[@name='warehouse_camera_optical_joint']")
        self.assertEqual(joint.find('parent').get('link'),'depth_cam_link')
    def test_docks_clear_physical_fixture(self):
        for x,y in self.config['racks'].values():
            dock=(x-self.config['dock_offset'],y)
            for _,p,size in fixture_parts(self.config):
                dx=max(abs(dock[0]-p[0])-size[0]/2,0)
                dy=max(abs(dock[1]-p[1])-size[1]/2,0)
                self.assertGreaterEqual((dx*dx+dy*dy)**.5,.16)
    def test_destination_slots_separate_and_supported(self):
        for output in self.config['outputs']:
            points=[slot(self.config,output,i) for i in range(len(self.config['output_offsets']))]
            self.assertTrue(all(inside(self.config,output,p) for p in points))
            for p,q in zip(points,points[1:]):self.assertGreater(np.linalg.norm(np.array(p[:2])-q[:2]),self.config['cargo_min_spacing'])
    def test_all_pick_and_place_docks_connected_in_safe_map(self):
        metadata=yaml.safe_load((SHARE/'config/warehouse_map.yaml').read_text())
        image=cv2.imread(str(SHARE/'config/warehouse_map.pgm'),cv2.IMREAD_GRAYSCALE)
        clearance=cv2.distanceTransform((image>200).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)*metadata['resolution']
        safe=clearance>.16
        def cell(position):
            col=int((position[0]-metadata['origin'][0])/metadata['resolution'])
            row=image.shape[0]-1-int((position[1]-metadata['origin'][1])/metadata['resolution'])
            return row,col
        first=cell(self.config['start']);seen={first};queue=deque([first])
        while queue:
            row,col=queue.popleft()
            for dr,dc in [(1,0),(-1,0),(0,1),(0,-1)]:
                nxt=(row+dr,col+dc)
                if 0<=nxt[0]<safe.shape[0] and 0<=nxt[1]<safe.shape[1] and safe[nxt] and nxt not in seen:
                    seen.add(nxt);queue.append(nxt)
        targets=[dock(self.config,g['position']) for g in validate(self.config)]
        targets.extend(dock(self.config,slot(self.config,out,i),self.config['outputs'][out]['center']) for out in self.config['outputs'] for i in range(len(self.config['output_offsets'])))
        for p in targets:self.assertTrue(cell(p) in seen,f'unreachable dock {p}')

if __name__=='__main__':unittest.main()
