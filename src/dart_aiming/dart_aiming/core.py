"""MCU output policy. All angles are relative, right-positive."""

import math
import numpy as np

INVALID, ACQUIRE, TRACKING, DEGRADED = range(4)


def command(
    mode,
    offset,
    launcher,
    base,
    center_light,
    light,
    *,
    clear,
    base_valid,
    light_valid,
    measured,
    conflict=False,
    allow_acquire=True
):
    if (
        mode not in (1, 2, 3, 4)
        or not math.isfinite(offset)
        or not clear
        or not base_valid
    ):
        return INVALID, 0.0, 0.0
    if mode in (3, 4):
        target, distance_point = base, center_light
        state = TRACKING if measured else DEGRADED
    elif light_valid and not conflict:
        target = distance_point = light
        state = TRACKING if measured else DEGRADED
    elif allow_acquire and not conflict:
        target = distance_point = center_light
        state = ACQUIRE
    else:
        return INVALID, 0.0, 0.0
    target, distance_point = np.asarray(target), np.asarray(distance_point)
    local = launcher[:3, :3].T @ (target - launcher[:3, 3])
    distance = float(np.linalg.norm(distance_point - launcher[:3, 3]))
    if (
        not np.isfinite(local).all()
        or not math.isfinite(distance)
        or distance <= 0
        or np.linalg.norm(local[:2]) < 1e-9
    ):
        return INVALID, 0.0, 0.0
    yaw = math.atan2(-local[1], local[0]) + offset
    return state, math.atan2(math.sin(yaw), math.cos(yaw)), distance
