"""Headless regression checks: python -m unittest test_module_grid.py."""
import argparse
from pathlib import Path
import unittest
import mujoco
import numpy as np
from module_grid import grid_xml, positive_int
from simulation_config import configured_xml, load_config
ROOT = Path(__file__).resolve().parent

class GridTests(unittest.TestCase):
    def model(self, w, h, **settings):
        config = load_config(ROOT / 'config.json') | settings
        return mujoco.MjModel.from_xml_string(grid_xml(configured_xml(ROOT / 'robot.xml', config), w, h))

    def test_grids(self):
        for w,h in ((1,1),(2,1),(1,2),(2,2),(3,2)):
            m=self.model(w,h);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            self.assertEqual(m.nu,16*w*h)
            self.assertEqual(m.npair,16*w*h)
            names=[m.actuator(i).name for i in range(m.nu)]
            self.assertEqual(len(set(names)),m.nu)
            self.assertIn('p0_m0' if w==h==1 else 'r0_c0_p0_m0',names)
            seams=[i for i in range(m.neq) if m.equality(i).name.startswith('seam_')]
            self.assertEqual(len(seams),4*((w-1)*h+(h-1)*w))
            for i in seams:
                np.testing.assert_allclose(d.site_xpos[m.eq_obj1id[i]],d.site_xpos[m.eq_obj2id[i]],atol=1e-10)

    def test_load_transfer(self):
        m=self.model(2,2);d=mujoco.MjData(m)
        for _ in range(600):mujoco.mj_step(m,d)
        d.xfrc_applied[m.body('r0_c0_panel_0').id,:3]=[.2,0,0]
        for _ in range(100):mujoco.mj_step(m,d)
        self.assertTrue(np.isfinite(d.qpos).all())
        self.assertFalse(any(w.number for w in d.warning))
        for i in range(m.neq):
            if m.equality(i).name.startswith('seam_'):
                self.assertLess(np.linalg.norm(d.site_xpos[m.eq_obj1id[i]]-d.site_xpos[m.eq_obj2id[i]]),1e-4)
        rows=[i for i in range(d.nefc) if d.efc_type[i]==mujoco.mjtConstraint.mjCNSTR_EQUALITY and m.equality(d.efc_id[i]).name.startswith('seam_')]
        self.assertGreater(np.max(np.abs(d.efc_force[rows])),1e-3)

    def test_fixed_base(self):
        free=self.model(2,2);fixed=self.model(2,2,fixed_base=True)
        self.assertEqual(free.nq-fixed.nq,7)
        self.assertEqual(sum(fixed.jnt_type==mujoco.mjtJoint.mjJNT_FREE),3)

    def test_invalid_dimensions(self):
        for value in ('0','-2','1.5','abc'):
            with self.assertRaises(argparse.ArgumentTypeError):positive_int(value)
        for w,h in ((0,1),(1,-1),(1.5,2),(True,2)):
            with self.assertRaises(ValueError):grid_xml('',w,h)

if __name__=='__main__':unittest.main()
