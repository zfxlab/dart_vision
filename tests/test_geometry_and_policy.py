import math
import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for package in ("dart_target_estimation", "dart_aiming"):
    sys.path.insert(0, str(ROOT / "src" / package))
from dart_target_estimation.core import VisibilityWindow, ray_rail, matrix, point, fresh
from dart_aiming.core import command, INVALID, ACQUIRE, TRACKING, DEGRADED


class GeometryTests(unittest.TestCase):
    def test_ray_recovers_offset_camera_target(self):
        zero = np.array([25.0, -0.28, 1.0])
        axis = np.array([0, 1, 0])
        origin = np.array([0.2, 0.03, 0.6])
        expected = zero + 0.43 * axis
        actual = ray_rail(origin, expected - origin, zero, axis)
        np.testing.assert_allclose(actual, expected, atol=1e-10)

    def test_parallel_behind_outside_and_residual_rejected(self):
        self.assertIsNone(ray_rail([0, 0, 0], [1, 0, 0], [25, 0, 0], [1, 0, 0]))
        self.assertIsNone(ray_rail([0, 0, 0], [-1, 0, 0], [25, 0, 0], [0, 1, 0]))
        self.assertIsNone(ray_rail([0, 0, 0], [25, 2, 0], [25, 0, 0], [0, 1, 0]))
        self.assertIsNone(ray_rail([0, 0, 0], [25, 0, 2], [25, 0, 0], [0, 1, 0]))

    def test_current_orientation_and_optical_sign(self):
        theta = -0.12
        launcher = matrix([0, 0, 0], [0, 0, math.sin(theta / 2), math.cos(theta / 2)])
        target = np.array([25, -2, 1])
        state, yaw, d = command(
            1,
            0.02,
            launcher,
            target,
            target,
            target,
            clear=True,
            base_valid=True,
            light_valid=True,
            measured=True,
        )
        self.assertEqual(state, TRACKING)
        self.assertAlmostEqual(yaw, -(math.atan2(-2, 25) - theta) + 0.02)
        self.assertAlmostEqual(d, math.sqrt(630))

    def test_camera_mount_rotation_is_in_transform(self):
        mount = matrix(
            [0.2, 0, 0.6], [0, 0, math.sin(-math.pi / 180), math.cos(-math.pi / 180)]
        )
        target = point(mount, [25, 0, 0])
        result = command(
            1,
            0,
            np.eye(4),
            target,
            target,
            target,
            clear=True,
            base_valid=True,
            light_valid=True,
            measured=True,
        )
        self.assertGreater(result[1], 0.03)

    def test_moving_mode_distance_is_virtual_center(self):
        base = [25, 0, 1]
        center = [25, 0, 1.2]
        options = dict(clear=True, base_valid=True, light_valid=True, measured=True)
        a = command(3, 0.01, np.eye(4), base, center, [25, -0.28, 1.2], **options)
        b = command(4, 0.01, np.eye(4), base, center, [25, 0.28, 1.2], **options)
        self.assertEqual(a, b)
        self.assertAlmostEqual(a[2], np.linalg.norm(center))
        self.assertAlmostEqual(a[1], 0.01)

    def test_modes_loss_defaults_and_conflict(self):
        common = dict(clear=True, base_valid=True, light_valid=False, measured=False)
        args = (np.eye(4), [25, 0, 1], [25, 0, 1.2], [0, 0, 0])
        self.assertEqual(command(1, 0, *args, **common)[0], ACQUIRE)
        self.assertEqual(command(3, 0, *args, **common)[0], DEGRADED)
        self.assertEqual(command(0, 0, *args, **common)[0], INVALID)
        self.assertEqual(command(1, 0, *args, conflict=True, **common)[0], INVALID)
        common["clear"] = False
        self.assertEqual(command(3, 0, *args, **common)[0], INVALID)

    def test_nonfinite_offset_rejected(self):
        self.assertEqual(
            command(
                1,
                float("nan"),
                np.eye(4),
                [25, 0, 1],
                [25, 0, 1],
                [25, 0, 1],
                clear=True,
                base_valid=True,
                light_valid=True,
                measured=True,
            )[0],
            INVALID,
        )

    def test_freshness(self):
        self.assertFalse(fresh(2, 1, 0.2))
        self.assertFalse(fresh(1, 2, 0.2))
        self.assertTrue(fresh(1, 1.1, 0.2))


class WindowTests(unittest.TestCase):
    def advance(self, w, state, start, count=8):
        for i in range(count):
            w.observe(state, start + i * 0.05)

    def test_repeated_open_and_unknown_do_not_restart(self):
        w = VisibilityWindow()
        self.advance(w, 2, 1)
        self.assertEqual(w.epoch, 1)
        w.observe(0, 1.5)
        self.advance(w, 2, 1.6)
        self.assertEqual(w.epoch, 1)
        self.advance(w, 1, 2.2)
        self.advance(w, 2, 2.7)
        self.assertEqual(w.epoch, 2)

    def test_single_frame_and_missing_frames_do_not_confirm(self):
        w = VisibilityWindow()
        w.observe(2, 1)
        w.observe(2, 2)
        self.assertEqual(w.epoch, 0)

    def test_stale_duplicate_and_out_of_order(self):
        w = VisibilityWindow()
        self.advance(w, 2, 1)
        w.observe(1, 1.2)
        self.assertEqual(w.state, 2)
        self.assertEqual(w.tick(2), 0)
        self.assertEqual(w.epoch, 1)

    def test_close_bounce_does_not_rearm(self):
        w = VisibilityWindow()
        self.advance(w, 2, 1)
        w.observe(1, 1.4)
        self.advance(w, 2, 1.45)
        self.assertEqual(w.epoch, 1)


if __name__ == "__main__":
    unittest.main()
