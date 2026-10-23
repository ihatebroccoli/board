"""Task semantics: attachment, proximity, incompatible sockets, stops, reset."""
import unittest
import numpy as np
from task_model import TaskModel, INSERTION_TASKS, pose, validate_layout

class TaskTests(unittest.TestCase):
    def setUp(self):
        self.m = TaskModel({k:[i*.1,0,.1] for i,k in enumerate(INSERTION_TASKS)},
                           {k:[i*.1,.3,.1] for i,k in enumerate(INSERTION_TASKS)})
    def test_pick_move_snap_and_unplug(self):
        tcp = self.m.plugs['usb'].copy()
        self.m.grasp('usb', tcp)
        tcp[2,3] += .12
        self.m.update(tcp)
        self.assertAlmostEqual(self.m.plugs['usb'][2,3], .22)
        with self.assertRaises(ValueError): self.m.insert('lan')
        with self.assertRaises(ValueError): self.m.insert('usb')
        tcp = self.m.targets['usb'].copy(); tcp[2,3] += .01
        self.m.update(tcp); self.m.insert('usb')
        self.assertIsNone(self.m.held)
        self.assertEqual(self.m.plug_state['usb'], 'inserted')
        np.testing.assert_allclose(self.m.plugs['usb'], self.m.targets['usb'])
        with self.assertRaises(ValueError): self.m.grasp('usb', tcp)
        self.m.unplug('usb',tcp); self.assertEqual(self.m.held,'usb')
    def test_stops_and_detent(self):
        self.m.rotate('rotary',79); self.assertFalse(self.m.rotary_on)
        self.m.rotate('rotary',81); self.assertEqual(self.m.angles['rotary'],105)
        self.m.rotate('rotary',74); self.assertTrue(self.m.rotary_on)
        self.m.rotate('rotary',69); self.assertEqual(self.m.angles['rotary'],45)
        self.assertEqual(sum(e['event']=='click' for e in self.m.events),2)
        for task,angle in [('rotary',106),('rotary',44),('locker',91),('locker',-1),('latch',180)]:
            with self.assertRaises(ValueError): self.m.rotate(task,angle)
    def test_grasp_rejects_distance_and_duplicate(self):
        with self.assertRaises(ValueError): self.m.grasp('bnc',pose([5,5,5]))
        self.m.grasp('bnc',self.m.plugs['bnc'])
        with self.assertRaises(ValueError): self.m.grasp('usb',self.m.plugs['usb'])
    def test_release_gravity_reset_and_relayout(self):
        self.m.grasp('usb',self.m.plugs['usb']); self.m.release()
        self.m.update(np.eye(4),1.)
        self.assertEqual(self.m.plug_state['usb'],'on_table')
        self.m.reset(); self.assertEqual(self.m.plug_state['usb'],'in_rack')
        self.m.grasp('usb',self.m.plugs['usb']); self.m.update(self.m.targets['usb']); self.m.insert('usb')
        self.m.translate_panel_target('usb',[.1,.1,0])
        np.testing.assert_allclose(self.m.plugs['usb'],self.m.targets['usb'])
        self.m.reset(); np.testing.assert_allclose(self.m.plugs['usb'],self.m.homes['usb'])
    def test_layout_requires_permutation(self):
        valid=[['usb','locker','outlet'],['audio','hdmi','latch'],['lan','rotary','bnc']]
        self.assertEqual(validate_layout(valid),valid)
        valid[0][0]='lan'
        with self.assertRaises(ValueError): validate_layout(valid)

if __name__=='__main__': unittest.main()
