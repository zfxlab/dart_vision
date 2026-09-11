import sys
import unittest
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/dart_lidar"))
from dart_lidar.core import (
    Accumulator,
    visibility,
    fit_base,
    fit_module,
    read_pcd,
    read_stl,
)


class LidarTests(unittest.TestCase):
    def test_actual_models_are_metres_and_nonempty(self):
        base = read_pcd(ROOT / "src/dart_bringup/models/pcd/base_pnx_5mm.pcd")
        module = read_stl(
            ROOT
            / "third_party/dart_description/meshes/pnx_dart_detection_module_link.stl"
        )
        self.assertGreater(len(base), 1000)
        self.assertGreater(len(module), 1000)
        self.assertLess(np.max(np.abs(base)), 5)
        self.assertLess(np.max(np.abs(module)), 2)

    def test_actual_base_translation_on_flat_surfaces(self):
        from dart_lidar.core import voxel

        model = voxel(read_pcd(ROOT / "src/dart_bringup/models/pcd/base_pnx_5mm.pcd"))
        shift = np.array([0.02, -0.015, 0.01])
        result = fit_base(model + shift, model)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result[0], shift, atol=0.005)

    def test_sparse_and_occluded_scene_rejected(self):
        self.assertIsNone(fit_base(np.zeros((5, 3)), np.zeros((5, 3))))
        self.assertIsNone(fit_module(np.zeros((5, 3)), np.zeros((5, 3))))

    def test_base_translation(self):
        rng = np.random.default_rng(3)
        model = rng.uniform(-0.5, 0.5, (1000, 3))
        shift = np.array([0.025, -0.018, 0.012])
        obs = model + shift + rng.normal(0, 0.001, model.shape)
        result = fit_base(obs, model)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result[0], shift, atol=0.005)

    def test_module_position_and_mixed_positions(self):
        rng = np.random.default_rng(5)
        model = rng.uniform([-0.04, -0.10, -0.1], [0.04, 0.10, 0.1], (800, 3))
        obs = model + np.array([0.28, 0, 0])
        result = fit_module(obs, model)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[0], 0.28, delta=0.011)
        mixed = np.concatenate([model + [0, 0, 0], model + [0.56, 0, 0]])
        self.assertIsNone(fit_module(mixed, model))

    def test_accumulator_expiry_and_reset(self):
        a = Accumulator(0.12, 1000)
        a.add(1, np.ones((1, 3)))
        a.add(1.2, np.zeros((1, 3)))
        self.assertEqual(a.get()[1], 1.2)
        a.clear()
        self.assertEqual(len(a.get()[0]), 0)

    def test_dense_accumulation_preserves_required_duration(self):
        rng = np.random.default_rng(9)
        a = Accumulator(0.8, 15000)
        for i in range(14):
            a.add(1 + i * 0.05, rng.uniform(-1, 1, (3000, 3)))
        points, start = a.get()
        self.assertGreaterEqual(1.65 - start, 0.5)
        self.assertLessEqual(len(points), 15000)

    def test_no_return_is_not_open(self):
        empty = np.empty((0, 3))
        self.assertEqual(
            visibility(empty, np.array([0.3, -1, -1]), np.array([2, 1, 1]), 0)[0], 0
        )
        self.assertEqual(
            visibility(empty, np.array([0.3, -1, -1]), np.array([2, 1, 1]), 20)[0], 2
        )

    def test_door_overrides_far_returns(self):
        rng = np.random.default_rng(4)
        door = rng.uniform([0.5, -0.3, -0.3], [0.6, 0.3, 0.3], (100, 3))
        self.assertEqual(
            visibility(door, np.array([0.3, -1, -1]), np.array([2, 1, 1]), 100)[0], 1
        )


if __name__ == "__main__":
    unittest.main()
