"""Regressions for the visible locker bolt, using OpenUSD without Isaac/RTX."""
import copy
import json
from pathlib import Path
import unittest

import numpy as np
from pxr import Usd, UsdGeom

from controller import SceneController


HERE = Path(__file__).resolve().parent
VENDOR = HERE.parents[1] / "logs/simulation/rb3_taskboard/vendor"
PANEL = "/World/Taskboard/Panels/locker"
TONGUE = PANEL + "/LatchTongue"


def world_pose(stage, path):
    return np.asarray(
        UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0)
    ).T


def world_vertices(stage, path):
    points = np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(path)).GetPointsAttr().Get())
    transform = world_pose(stage, path)
    return points @ transform[:3, :3].T + transform[:3, 3]


class LockerLinkageTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        config = json.loads((HERE / "config/scene.json").read_text())
        # The reconstructed fixture is rotated relative to the world axes.
        # A world-X translation would move the bolt sideways out of its housing.
        config.setdefault("fixture_yaw_deg", {})["locker"] = 90.
        self.scene = SceneController(self.stage, config, VENDOR, fps=2)
        origin = np.asarray(self.scene.handles["rotary_origins"]["locker"])
        self.scene.move_tcp(origin + [0, 0, .025], duration=.1)

    def test_handle_rotation_retracts_tongue_and_reverse_restores_it(self):
        c = self.scene
        fixed_mount = world_vertices(self.stage, PANEL + "/MountPlate")
        original = world_vertices(self.stage, TONGUE)
        local_x_in_world = world_pose(self.stage, PANEL)[:3, 0]
        np.testing.assert_allclose(local_x_in_world, [0., 1., 0.], atol=1e-12)
        # Track an actual vertex at the extended tip, not the mesh's transform
        # origin: this mesh stores its original position in its point coordinates.
        tip = int(np.argmax(original @ local_x_in_world))
        extensions = []
        snapshots = []
        for angle in (0., 45., 90., 45., 0.):
            c.rotate("locker", angle, duration=.1)
            current = world_vertices(self.stage, TONGUE)
            displacement = current[tip] - original[tip]
            axial = float(displacement @ local_x_in_world)
            # All vertices must translate together, keeping the bolt aligned
            # with the fixed housing while the handle itself rotates.
            np.testing.assert_allclose(
                current - original,
                np.broadcast_to(axial * local_x_in_world, original.shape),
                atol=1e-9,
            )
            np.testing.assert_allclose(
                world_vertices(self.stage, PANEL + "/MountPlate"), fixed_mount,
                atol=1e-9,
            )
            extensions.append(axial)
            snapshots.append(current)

        self.assertLess(extensions[1], extensions[0])
        self.assertLess(extensions[2], extensions[1])
        self.assertLess(extensions[2], -.010)
        np.testing.assert_allclose(snapshots[3], snapshots[1], atol=1e-9)
        np.testing.assert_allclose(snapshots[4], original, atol=1e-9)

    def test_rearrange_preserves_retraction_and_reset_extends_at_new_slot(self):
        c = self.scene
        extended = world_vertices(self.stage, TONGUE)
        c.rotate("locker", 90., duration=.1)
        retracted = world_vertices(self.stage, TONGUE)
        panel_before = world_pose(self.stage, PANEL)
        layout = copy.deepcopy(c.layout)
        layout[0][0], layout[0][1] = layout[0][1], layout[0][0]
        c.rearrange(layout)
        panel_delta = world_pose(self.stage, PANEL)[:3, 3] - panel_before[:3, 3]
        self.assertGreater(np.linalg.norm(panel_delta), .08)
        self.assertEqual(c.model.angles["locker"], 90.)
        np.testing.assert_allclose(
            world_vertices(self.stage, TONGUE), retracted + panel_delta, atol=1e-9,
        )

        c.reset()
        self.assertEqual(c.layout, layout)
        self.assertEqual(c.model.angles["locker"], 0.)
        np.testing.assert_allclose(
            world_vertices(self.stage, TONGUE), extended + panel_delta, atol=1e-9,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
