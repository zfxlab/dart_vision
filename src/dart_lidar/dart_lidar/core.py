"""Bounded, robust point-cloud estimators. Units: metres."""

from collections import deque
import numpy as np
from scipy.spatial import cKDTree


def voxel(points, size=0.02, limit=12000):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return points
    _, ids = np.unique(
        np.floor(points / size).astype(np.int64), axis=0, return_index=True
    )
    out = points[np.sort(ids)]
    if len(out) > limit:
        out = out[np.linspace(0, len(out) - 1, limit, dtype=int)]
    return out


def crop(points, lower, upper):
    return points[np.all((points >= lower) & (points <= upper), axis=1)]


def read_pcd(path):
    with open(path, "rb") as f:
        fields = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError("Missing PCD data")
            parts = line.decode("ascii").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            fields[parts[0]] = parts[1:]
            if parts[0] == "DATA":
                break
        if fields["DATA"] == ["ascii"]:
            data = np.loadtxt(f, ndmin=2)
            return data[:, [fields["FIELDS"].index(x) for x in ("x", "y", "z")]]
        if fields["DATA"] != ["binary"]:
            raise ValueError("Only binary and ASCII PCD supported")
        counts = fields.get("COUNT", ["1"] * len(fields["FIELDS"]))
        dtype = np.dtype(
            [
                (name, {"F": "<f", "I": "<i", "U": "<u"}[kind] + size, (int(count),))
                for name, kind, size, count in zip(
                    fields["FIELDS"], fields["TYPE"], fields["SIZE"], counts
                )
            ]
        )
        data = np.frombuffer(f.read(), dtype=dtype)
        return np.column_stack([data[x][:, 0] for x in ("x", "y", "z")])


def read_stl(path, count=6000):
    with open(path, "rb") as f:
        header = f.read(80)
        n = int.from_bytes(f.read(4), "little")
        raw = f.read()
    if len(raw) != n * 50:
        raise ValueError("Expected binary STL in metres")
    data = np.frombuffer(
        raw,
        dtype=np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]),
    )
    v = data["v"].astype(float)
    areas = np.linalg.norm(np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0]), axis=1)
    if areas.sum() <= 0:
        raise ValueError("Empty mesh")
    rng = np.random.default_rng(7)
    triangles = v[rng.choice(n, count, p=areas / areas.sum())]
    u = np.sqrt(rng.random(count))
    w = rng.random(count)
    return (
        (1 - u[:, None]) * triangles[:, 0]
        + (u * (1 - w))[:, None] * triangles[:, 1]
        + (u * w)[:, None] * triangles[:, 2]
    )


class Accumulator:
    def __init__(self, seconds, max_points):
        self.seconds = seconds
        self.max_points = max_points
        self.frames = deque()

    def clear(self):
        self.frames.clear()

    def add(self, stamp, points):
        if self.frames and stamp <= self.frames[-1][0]:
            self.clear()
        self.frames.append((stamp, voxel(points)))
        while self.frames and stamp - self.frames[0][0] > self.seconds:
            self.frames.popleft()
        # Preserve the time span when memory is capped. Dropping whole frames can
        # make a minimum-duration initializer impossible on dense scans.
        if sum(len(p) for _, p in self.frames) > self.max_points:
            budget = max(1, self.max_points // len(self.frames))
            self.frames = deque(
                (
                    t,
                    (
                        p
                        if len(p) <= budget
                        else p[np.linspace(0, len(p) - 1, budget, dtype=int)]
                    ),
                )
                for t, p in self.frames
            )

    def get(self):
        if not self.frames:
            return np.empty((0, 3)), 0.0
        return (
            voxel(np.concatenate([p for _, p in self.frames]), limit=self.max_points),
            self.frames[0][0],
        )


def fit_base(
    points, model, max_shift=0.3, threshold=0.08, min_support=0.65, min_points=80
):
    """Translation-only trimmed ICP: retain the startup orientation prior."""
    points = voxel(points)
    if len(points) < min_points:
        return None
    tree = cKDTree(model)
    shift = np.zeros(3)
    for _ in range(30):
        distance, ids = tree.query(points - shift)
        mask = distance < max(threshold * 3, 0.2)
        if mask.sum() < min_points:
            return None
        # Trim the largest correspondence errors, then minimize squared residuals.
        # Coordinate-wise medians can stick tangentially on large flat surfaces.
        cutoff = np.quantile(distance[mask], 0.90)
        trimmed = mask & (distance <= cutoff)
        residual = points[trimmed] - model[ids[trimmed]]
        updated = np.mean(residual, axis=0)
        if np.linalg.norm(updated) > max_shift:
            return None
        if np.linalg.norm(updated - shift) < 0.00005:
            shift = updated
            break
        shift = updated
    distances, _ = tree.query(points - shift)
    good = distances < threshold
    support = float(good.mean())
    if good.sum() < min_points or support < min_support:
        return None
    # Reject a tiny fragment masquerading as an entire base.
    spread = np.ptp(points[good], axis=0)
    if np.count_nonzero(spread > 0.20) < 2:
        return None
    rmse = float(np.sqrt(np.mean(distances[good] ** 2)))
    if rmse > threshold * 0.65:
        return None
    return shift, rmse, support


def fit_module(
    points,
    model,
    step=0.01,
    threshold=0.035,
    min_points=12,
    min_support=0.55,
    ambiguity_margin=0.002,
):
    """Search the entire rail every time, independent of competition mode."""
    points = voxel(points, size=0.01, limit=1000)
    if len(points) < min_points:
        return None
    tree = cKDTree(model)
    qs = np.linspace(0, 0.56, int(round(0.56 / step)) + 1)
    scores = []
    supports = []
    errors = []
    for q in qs:
        d, _ = tree.query(points - np.array([q, 0, 0]))
        good = d < threshold
        supports.append(float(good.mean()))
        errors.append(
            float(np.sqrt(np.mean(d[good] ** 2))) if good.any() else float("inf")
        )
        scores.append(float(np.mean(np.minimum(d, threshold * 3))))
    i = int(np.argmin(scores))
    separated = np.abs(qs - qs[i]) > 0.04
    if (
        supports[i] < min_support
        or supports[i] * len(points) < min_points
        or errors[i] > threshold * 0.7
    ):
        return None
    if (
        separated.any()
        and np.min(np.asarray(scores)[separated]) - scores[i] < ambiguity_margin
    ):
        return None
    return float(qs[i]), errors[i], supports[i]


def visibility(points, near_lower, near_upper, far_points, min_near=15, min_far=10):
    near = crop(points, near_lower, near_upper)
    # Spatial support is needed: one noisy return does not prove a door.
    blocked = (
        len(near) >= min_near and np.count_nonzero(np.ptp(near, axis=0) > 0.10) >= 2
    )
    if blocked:
        return 1, len(near), far_points
    if len(near) == 0 and far_points >= min_far:
        return 2, 0, far_points
    return 0, len(near), far_points
