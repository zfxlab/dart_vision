from .core import matrix


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def xyz(p):
    return [p.x, p.y, p.z]


def transform(t):
    return matrix(
        xyz(t.translation), [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
    )


def pose_matrix(p):
    return matrix(
        xyz(p.position),
        [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
    )


def fill_point(out, value):
    out.x, out.y, out.z = map(float, value)
