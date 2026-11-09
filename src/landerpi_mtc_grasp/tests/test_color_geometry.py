import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from color_geometry import locate, transform_cloud, upright_center, colored_cloud


class ColorGeometryTest(unittest.TestCase):
    def setUp(self):
        self.rgb = np.zeros((80, 120, 3), np.uint8)
        self.rgb[30:50, 50:70] = (0, 0, 255)
        self.rgb[10:25, 10:25] = (0, 255, 0)
        self.depth = np.full((80, 120), .8, np.float32)
        self.k = [100., 0., 60., 0., 100., 40., 0., 0., 1.]

    def test_requested_color_and_projection(self):
        red, _, _ = locate(self.rgb, self.depth, '32FC1', self.k, 'red')
        green, _, _ = locate(self.rgb, self.depth, '32FC1', self.k, 'green')
        np.testing.assert_allclose(red, [-.008, -.008, .8], atol=1e-6)
        self.assertLess(green[0], -.3)
        self.assertLess(green[1], -.1)

    def test_millimeter_depth_matches_float_depth(self):
        meters, _, _ = locate(self.rgb, self.depth, '32FC1', self.k, 'red')
        millimeters, _, _ = locate(self.rgb, np.full((80,120),800,np.uint16), '16UC1', self.k, 'red')
        np.testing.assert_allclose(meters, millimeters, atol=1e-6)

    def test_missing_color_and_invalid_depth_fail(self):
        with self.assertRaises(ValueError):
            locate(self.rgb, self.depth, '32FC1', self.k, 'blue')
        for invalid in (0., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                locate(self.rgb, np.full_like(self.depth, invalid), '32FC1', self.k, 'red')

    def test_invalid_intrinsics_or_encoding_fail(self):
        for encoding, k in [('bad',self.k),('32FC1',[0.]*9)]:
            with self.assertRaises(ValueError):
                locate(self.rgb, self.depth, encoding, k, 'red')

    def test_oblique_cloud_top_center_ignores_visible_side(self):
        x, y = np.meshgrid(np.linspace(.22,.26,20), np.linspace(-.02,.02,20))
        top = np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.05)))
        y, z = np.meshgrid(np.linspace(-.02,.02,20),np.linspace(.001,.049,20))
        side = np.column_stack((np.full(y.size,.22),y.ravel(),z.ravel()))
        np.testing.assert_allclose(upright_center(np.vstack((top,side))), [.24,0.,.025],atol=.002)
        with self.assertRaises(ValueError):
            upright_center(top+[0.,0.,.16])

    def test_rigid_camera_transform_and_registered_cloud_units(self):
        points=np.array([[1.,0.,0.],[0.,1.,0.]])
        result=transform_cloud(points,[.2,.3,.4],[0.,0.,np.sin(np.pi/4),np.cos(np.pi/4)])
        np.testing.assert_allclose(result,[[.2,1.3,.4],[-.8,.3,.4]],atol=1e-9)
        cloud,_=colored_cloud(self.rgb,self.depth,'32FC1',self.k,'red')
        mm_cloud,_=colored_cloud(self.rgb,np.full((80,120),800,np.uint16),'16UC1',self.k,'red')
        np.testing.assert_allclose(cloud,mm_cloud,atol=1e-6)


if __name__ == '__main__':
    unittest.main()
