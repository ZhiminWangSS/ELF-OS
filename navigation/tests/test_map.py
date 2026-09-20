import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from pcd import read_pcd,write_pcd
from prepare_map import fit_floor,level_rotation,ray_cells

class MapTests(unittest.TestCase):
    def test_binary_roundtrip_and_truncation(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'p.pcd'; points=np.array([[1,2,3,4],[4,5,6,7]],dtype=np.float32)
            write_pcd(p,points);np.testing.assert_equal(read_pcd(p),points)
            p.write_bytes(p.read_bytes()[:-1])
            with self.assertRaises(ValueError):read_pcd(p)
    def test_tilted_floor_with_ceiling_and_obstacles(self):
        rng=np.random.RandomState(7)
        xy=rng.uniform(-10,10,(5000,2));floor=np.c_[xy,rng.normal(0,.015,5000)]
        ceiling=floor+[0,0,3];objects=rng.uniform([-10,-10,0],[10,10,3],(2000,3))
        trajectory=np.c_[rng.uniform(-8,8,(200,2)),np.full(200,.48)]
        angle=.5;R=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
        cloud=np.r_[floor,ceiling,objects]@R.T;path=trajectory@R.T
        n,offset,q=fit_floor(cloud,path);leveled=cloud@level_rotation(n).T;leveled[:,2]-=offset
        self.assertLess(np.std(leveled[:5000,2]),.02)
        self.assertAlmostEqual(np.median(path@n-offset),.48,places=2)
    def test_ray_includes_intervening_wall(self):
        cells=ray_cells(np.array([.1,.1]),np.array([10.9,.1]));self.assertIn([5,0],cells.tolist())
        self.assertEqual(cells[-1].tolist(),[10,0])
if __name__=='__main__':unittest.main()
