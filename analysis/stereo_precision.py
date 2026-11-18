"""Pixel-noise propagation through the runtime two-ray triangulation geometry.

Camera poses map optical coordinates into the launcher frame. Distortion uses
the five-coefficient plumb_bob convention from the project's CameraInfo files.
Only NumPy is required, so the analysis does not depend on ROS OpenCV binaries.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraModel:
    pose: np.ndarray
    matrix: np.ndarray
    distortion: np.ndarray


def distort(point, coefficients):
    x, y = point
    k1, k2, p1, p2, k3 = coefficients
    r2 = x * x + y * y
    radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
    return np.array(
        [
            x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
            y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y,
        ]
    )


def project(camera, point):
    optical = camera.pose[:3, :3].T @ (point - camera.pose[:3, 3])
    if optical[2] <= 0:
        raise ValueError("Target is behind the camera")
    pixel = camera.matrix @ np.r_[distort(optical[:2] / optical[2], camera.distortion), 1.0]
    return pixel[:2] / pixel[2]


def bearing(camera, pixel):
    normalized = np.linalg.solve(camera.matrix, np.r_[pixel, 1.0])
    observed = normalized[:2] / normalized[2]
    point = observed.copy()
    # Newton inversion in the calibrated field of view; reject unconverged rays.
    for _ in range(20):
        residual = distort(point, camera.distortion) - observed
        if np.linalg.norm(residual, ord=np.inf) < 1e-13:
            optical = np.r_[point, 1.0]
            return camera.pose[:3, :3] @ (optical / np.linalg.norm(optical))
        x, y = point
        k1, k2, p1, p2, k3 = camera.distortion
        r2 = x * x + y * y
        radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
        gradient = 2 * (k1 + 2 * k2 * r2 + 3 * k3 * r2 * r2) * point
        jacobian = np.array(
            [
                [
                    radial + x * gradient[0] + 2 * p1 * y + 6 * p2 * x,
                    x * gradient[1] + 2 * p1 * x + 2 * p2 * y,
                ],
                [
                    y * gradient[0] + 2 * p1 * x + 2 * p2 * y,
                    radial + y * gradient[1] + 6 * p1 * y + 2 * p2 * x,
                ],
            ]
        )
        point -= np.linalg.solve(jacobian, residual)
    raise ValueError("Distortion inversion did not converge")


def triangulate(cameras, pixels):
    top, bottom = cameras
    dt, db = [bearing(camera, pixel) for camera, pixel in zip(cameras, pixels, strict=True)]
    ot, ob = top.pose[:3, 3], bottom.pose[:3, 3]
    cross = np.cross(dt, db)
    denominator = cross @ cross
    if denominator < 1e-24:
        raise ValueError("Degenerate stereo rays")
    offset = ob - ot
    st = np.cross(offset, db) @ cross / denominator
    sb = np.cross(offset, dt) @ cross / denominator
    if min(st, sb) <= 0:
        raise ValueError("Intersection is behind a camera")
    return (ot + st * dt + ob + sb * db) / 2


def pixel_jacobian(cameras, point, step_px=1e-3):
    """Return d(X,Y,Z)/d(u_top,v_top,u_bottom,v_bottom) in m/px."""
    pixels = np.array([project(camera, point) for camera in cameras])
    jacobian = np.empty((3, 4))
    for index in range(4):
        delta = np.zeros((2, 2))
        delta.flat[index] = step_px
        jacobian[:, index] = (
            triangulate(cameras, pixels + delta) - triangulate(cameras, pixels - delta)
        ) / (2 * step_px)
    return jacobian


def measurement_jacobian(cameras, point):
    """Return derivatives of launcher yaw, horizontal range and height vs pixels."""
    x, y, _ = point
    distance = np.hypot(x, y)
    if distance <= 0:
        raise ValueError("Target horizontal range must be positive")
    derivative = np.array(
        [
            [-y / distance**2, x / distance**2, 0],
            [x / distance, y / distance, 0],
            [0, 0, 1],
        ]
    )
    return derivative @ pixel_jacobian(cameras, point)
