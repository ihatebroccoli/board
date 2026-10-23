"""OpenUSD integration checks for reconstructed geometry, robot IK, and tasks.

Run with the separate inspection_env/bin/python; no Isaac/RTX process is needed.
The scene is built in memory and exercises the public SceneController methods.
"""
import copy
import json
from pathlib import Path
import unittest

import numpy as np
from pxr import Usd, UsdGeom

from controller import SceneController
from task_model import INSERTION_TASKS

HERE = Path(__file__).resolve().parent
VENDOR = HERE.parents[1] / "logs/simulation/rb3_taskboard/vendor"


def world_pose(stage, path):
    return np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)).T


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.config = json.loads((HERE / "config/scene.json").read_text())
        self.scene = SceneController(self.stage, self.config, VENDOR, fps=2, on_frame=None)

    def near_handle(self, name):
        position = np.asarray(self.scene.handles["rotary_origins"][name], dtype=float)
        self.scene.move_tcp(position + [0, 0, .025], duration=.1)

    def test_constructed_mesh_robot_tcp_and_plug_poses(self):
        c = self.scene
        self.assertEqual(len(c.handles["panel_paths"]), 9)
        self.assertEqual(len(c.handles["plug_paths"]), 6)
        self.assertEqual(len(c.handles["rotary_paths"]), 3)
        self.assertGreater(sum(v["triangles"] for v in c.robot.asset_stats.values()), 100_000)
        np.testing.assert_allclose(world_pose(self.stage, c.robot.tcp_path), c.robot.get_tcp_pose(), atol=1e-10)
        for name in INSERTION_TASKS:
            np.testing.assert_allclose(world_pose(self.stage, c.handles["plug_paths"][name]), c.model.homes[name], atol=1e-7)

    def test_rack_pick_carry_rejects_wrong_socket_and_distant_insert(self):
        c = self.scene
        home = c.model.homes["usb"][:3, 3].copy()
        c.move_tcp(home, duration=.1)
        c.grasp("usb")
        held_relative = np.linalg.inv(c.robot.get_tcp_pose()) @ c.model.plugs["usb"]
        c.move_tcp(home + [0, 0, .12], duration=.1)
        np.testing.assert_allclose(c.model.plugs["usb"], c.robot.get_tcp_pose() @ held_relative, atol=1e-9)
        self.assertGreater(c.model.plugs["usb"][2, 3] - home[2], .115)
        with self.assertRaises(ValueError):
            c.insert("usb")
        c.move_tcp(c.model.targets["lan"][:3, 3] + [0, 0, .01], duration=.1)
        before = c.model.plugs["usb"].copy()
        with self.assertRaises(ValueError):
            c.insert("lan")
        self.assertEqual(c.model.held, "usb")
        np.testing.assert_array_equal(c.model.plugs["usb"], before)
        c.release()
        c.hold(.5)
        self.assertIsNone(c.model.held)
        self.assertEqual(c.model.plug_state["usb"], "on_table")

    def test_all_six_rack_to_socket_paths_are_reachable(self):
        c = self.scene
        for name in INSERTION_TASKS:
            with self.subTest(task=name):
                c.pick_and_insert(name)
                self.assertEqual(c.model.plug_state[name], "inserted")
                self.assertIsNone(c.model.held)
                np.testing.assert_allclose(c.model.plugs[name], c.model.targets[name], atol=1e-10)
                np.testing.assert_allclose(world_pose(self.stage, c.handles["plug_paths"][name]), c.model.targets[name], atol=1e-7)
        self.assertTrue(c.ik_log)
        self.assertTrue(all(entry["success"] for entry in c.ik_log))
        self.assertLess(max(entry["position_error_m"] for entry in c.ik_log), .001)

    def test_rotary_proximity_and_bidirectional_detents(self):
        c = self.scene
        with self.assertRaises(ValueError):
            c.rotate("rotary", 81, duration=.1)
        self.near_handle("rotary")
        self.assertEqual(c.rotate("rotary", 81, duration=.1), 105.)
        self.assertTrue(c.model.rotary_on)
        self.assertEqual(c.rotate("rotary", 69, duration=.1), 45.)
        self.assertFalse(c.model.rotary_on)
        self.assertEqual(sum(event["event"] == "click" for event in c.model.events), 2)

    def test_rotation_targets_outside_stops_reject_before_animation(self):
        c = self.scene
        for name, targets in (("rotary", (44., 106.)), ("locker", (-1., 91.)), ("latch", (-166., 166.))):
            self.near_handle(name)
            for target in targets:
                with self.subTest(task=name, angle=target):
                    before = c.model.angles[name]
                    # Four frames expose invalid requests that would otherwise
                    # terminate early at a rotary detent before validation.
                    with self.assertRaises(ValueError):
                        c.rotate(name, target, duration=2.)
                    self.assertEqual(c.model.angles[name], before)

    def test_rotation_rejects_held_connector(self):
        c = self.scene
        c.move_tcp(c.model.homes["usb"][:3, 3], duration=.1)
        c.grasp("usb")
        self.near_handle("locker")
        with self.assertRaises(ValueError):
            c.rotate("locker", 90, duration=.1)
        self.assertEqual(c.model.held, "usb")

    def test_locker_and_latch_reach_their_valid_stops(self):
        c = self.scene
        for name, angles in (("locker", (90., 0.)), ("latch", (-165., 165., 0.))):
            self.near_handle(name)
            for angle in angles:
                with self.subTest(task=name, angle=angle):
                    self.assertEqual(c.rotate(name, angle, duration=.1), angle)
                    self.assertEqual(c.model.angles[name], angle)

    def test_rearrange_moves_modules_targets_handles_and_seated_plugs(self):
        c = self.scene
        c.pick_and_insert("usb")
        c.pick_and_insert("outlet")
        old_layout = copy.deepcopy(c.layout)
        old_panel = {key: value.copy() for key, value in c.panel_poses.items()}
        old_rotary = {key: value.copy() for key, value in c.rotary_base.items()}
        old_targets = {key: value.copy() for key, value in c.model.targets.items()}
        old_homes = {key: value.copy() for key, value in c.model.homes.items()}
        old_plugs = {key: value.copy() for key, value in c.model.plugs.items()}
        flat = [name for row in old_layout for name in row]
        rotated = flat[1:] + flat[:1]
        layout = [rotated[index:index + 3] for index in range(0, 9, 3)]
        c.rearrange(layout)
        self.assertEqual(c.layout, layout)
        for row in range(3):
            for col in range(3):
                name = layout[row][col]
                expected_position = old_panel[old_layout[row][col]][:3, 3]
                delta = expected_position - old_panel[name][:3, 3]
                np.testing.assert_allclose(world_pose(self.stage, c.handles["panel_paths"][name])[:3, 3], expected_position)
                if name in old_rotary:
                    expected = old_rotary[name][:3, 3] + delta
                    np.testing.assert_allclose(world_pose(self.stage, c.handles["rotary_paths"][name])[:3, 3], expected)
                    np.testing.assert_allclose(c.handles["rotary_origins"][name], expected)
                if name in old_targets:
                    np.testing.assert_allclose(c.model.targets[name][:3, 3], old_targets[name][:3, 3] + delta)
                    np.testing.assert_array_equal(c.model.homes[name], old_homes[name])
                    expected = old_plugs[name].copy()
                    if c.model.plug_state[name] == "inserted":
                        expected[:3, 3] += delta
                    np.testing.assert_allclose(world_pose(self.stage, c.handles["plug_paths"][name]), expected, atol=1e-7)
        c.reset()
        self.assertEqual(c.layout, layout)
        self.assertEqual(c.model.angles, {"rotary": 45., "latch": 0., "locker": 0.})
        for name in INSERTION_TASKS:
            self.assertEqual(c.model.plug_state[name], "in_rack")
            np.testing.assert_allclose(world_pose(self.stage, c.handles["plug_paths"][name]), old_homes[name], atol=1e-7)

    def test_nonfinite_tcp_rejected_without_state_changes(self):
        model = self.scene.model
        bad = np.eye(4)
        bad[0, 3] = np.nan
        with self.assertRaises(ValueError):
            model.grasp("usb", bad)
        self.assertIsNone(model.held)
        before_time = model.time
        before_plug = model.plugs["usb"].copy()
        with self.assertRaises(ValueError):
            model.update(bad, dt=.5)
        self.assertEqual(model.time, before_time)
        np.testing.assert_array_equal(model.plugs["usb"], before_plug)
        model.grasp("usb", model.homes["usb"])
        model.update(model.targets["usb"])
        model.insert("usb")
        with self.assertRaises(ValueError):
            model.unplug("usb", bad)
        self.assertIsNone(model.held)
        self.assertEqual(model.plug_state["usb"], "inserted")


if __name__ == "__main__":
    unittest.main(verbosity=2)
