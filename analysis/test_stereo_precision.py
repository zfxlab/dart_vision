"""Run with python -m unittest discover -s analysis -p test_stereo_precision.py."""

import unittest

import numpy as np
from stereo_precision import CameraModel, measurement_jacobian, project, triangulate


class StereoPrecisionTest(unittest.TestCase):
    def cameras(self, distorted=False):
        rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        matrix = np.diag([1500.0, 1500.0, 1.0])
        matrix[:2, 2] = [720, 540]
        distortion = np.array([-0.2, 0.05, 0.001, -0.002, 0.01]) if distorted else np.zeros(5)
        cameras = []
        for origin in ([0, 0.1, 0], [0, -0.1, 0]):
            pose = np.eye(4)
            pose[:3, :3] = rotation
            pose[:3, 3] = origin
            cameras.append(CameraModel(pose, matrix, distortion))
        return cameras

    def test_distorted_pixels_recover_off_axis_points(self):
        cameras = self.cameras(distorted=True)
        angle = 0.03
        pitch = np.array(
            [[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]]
        )
        cameras[1].pose[:3, :3] = pitch @ cameras[1].pose[:3, :3]
        cameras[1].pose[:3, 3] = [0.25, -0.1, -0.4]
        for point in (np.array([10.0, 0, 0]), np.array([20.0, 1.5, -0.8])):
            pixels = [project(camera, point) for camera in cameras]
            np.testing.assert_allclose(triangulate(cameras, pixels), point, atol=1e-9)

    def test_horizontal_pair_matches_analytic_range_noise(self):
        jacobian = measurement_jacobian(self.cameras(), np.array([10.0, 0, 0]))
        expected = np.sqrt(2) * 10.0**2 / (1500 * 0.2)
        self.assertAlmostEqual(np.linalg.norm(jacobian[1]), expected, places=7)
        self.assertAlmostEqual(np.linalg.norm(jacobian[0]), 1 / (np.sqrt(2) * 1500), places=9)

    def test_pixel_covariance_matches_monte_carlo(self):
        cameras = self.cameras(distorted=True)
        point = np.array([12.0, 0.5, 0.2])
        sigma = 0.02
        jacobian = measurement_jacobian(cameras, point)
        predicted = sigma * np.linalg.norm(jacobian, axis=1)
        pixels = np.array([project(camera, point) for camera in cameras])
        rng = np.random.default_rng(1234)
        samples = []
        for noise in rng.normal(0, sigma, (1000, 2, 2)):
            x, y, z = triangulate(cameras, pixels + noise)
            samples.append([np.arctan2(y, x), np.hypot(x, y), z])
        np.testing.assert_allclose(np.std(samples, axis=0), predicted, rtol=0.1)


if __name__ == "__main__":
    unittest.main()
