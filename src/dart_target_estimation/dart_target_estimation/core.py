"""Geometry and observation-window logic, independent of ROS."""

import math
import numpy as np


def fresh(stamp, now, timeout):
    return math.isfinite(stamp) and 0.0 <= now - stamp <= timeout


def matrix(translation, quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    norm = x * x + y * y + z * z + w * w
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid rotation")
    x, y, z, w = np.array([x, y, z, w]) / math.sqrt(norm)
    t = np.eye(4)
    t[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    t[:3, 3] = translation
    if not np.isfinite(t).all():
        raise ValueError("Nonfinite transform")
    return t


def point(t, p):
    return t[:3, :3] @ np.asarray(p) + t[:3, 3]


def ray_rail(
    origin, direction, zero, axis, bounds=(0.0, 0.56), max_residual=0.06, min_sine=0.05
):
    d = np.asarray(direction, dtype=float)
    a = np.asarray(axis, dtype=float)
    if np.linalg.norm(d) < 1e-9 or np.linalg.norm(a) < 1e-9:
        return None
    d, a = d / np.linalg.norm(d), a / np.linalg.norm(a)
    if not np.isfinite(d).all() or np.linalg.norm(np.cross(d, a)) < min_sine:
        return None
    design = np.column_stack((d, -a))
    depth, q = np.linalg.lstsq(design, np.asarray(zero) - origin, rcond=None)[0]
    residual = np.linalg.norm(
        np.asarray(origin) + depth * d - (np.asarray(zero) + q * a)
    )
    if depth <= 0 or not bounds[0] <= q <= bounds[1] or residual > max_residual:
        return None
    return np.asarray(zero) + q * a


class VisibilityWindow:
    UNKNOWN, BLOCKED, CLEAR = 0, 1, 2

    def __init__(self, confirm_open=0.25, confirm_close=0.2, timeout=0.3):
        if any(
            not math.isfinite(v) or v <= 0
            for v in (confirm_open, confirm_close, timeout)
        ):
            raise ValueError("Visibility durations must be positive")
        self.open_delay, self.close_delay, self.timeout = (
            confirm_open,
            confirm_close,
            timeout,
        )
        self.state = self.UNKNOWN
        self.epoch = 0
        self.opened_at = 0.0
        self.armed = True
        self.candidate = self.UNKNOWN
        self.since = None
        self.last = None

    def observe(self, evidence, stamp):
        if self.last is not None and stamp <= self.last:
            return
        # Missing frames must not count as continuous evidence.
        if self.last is None or stamp - self.last > self.timeout:
            self.candidate, self.since = self.UNKNOWN, None
        self.last = stamp
        if evidence == self.UNKNOWN:
            self.state, self.candidate, self.since = self.UNKNOWN, self.UNKNOWN, None
            return
        if evidence != self.candidate:
            self.candidate, self.since = evidence, stamp
        delay = self.open_delay if evidence == self.CLEAR else self.close_delay
        if stamp - self.since < delay:
            return
        self.state = evidence
        if evidence == self.BLOCKED:
            self.armed = True
        elif self.armed:
            self.epoch += 1
            self.opened_at = stamp
            self.armed = False

    def tick(self, now):
        if self.last is None or not fresh(self.last, now, self.timeout):
            self.state, self.candidate, self.since = self.UNKNOWN, self.UNKNOWN, None
        return self.state
