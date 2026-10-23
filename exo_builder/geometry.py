"""Small numpy <-> OpenSim helpers and rigid-body mass properties.

Everything here was checked against OpenSim 4.5 and 4.6 (see
tests/test_api_contract.py); prefer these helpers over ad-hoc conversions.
"""
import numpy as np
import opensim as osim


def vec3(v):
    return osim.Vec3(float(v[0]), float(v[1]), float(v[2]))


def to_np(v):
    """Vec3 / Vec6 / Vector -> 1D numpy array."""
    n = v.size() if hasattr(v, "size") else 3
    return np.array([v.get(i) for i in range(n)])


def rot_to_np(R):
    return np.array([[R.get(i, j) for j in range(3)] for i in range(3)])


def transform_np(frame, state):
    """(R, p) of a frame in ground as numpy arrays."""
    T = frame.getTransformInGround(state)
    return rot_to_np(T.R()), to_np(T.p())


def body_xyz_angles(R):
    """Body-fixed X-Y-Z Euler angles (OpenSim 'orientation' properties) of a rotation matrix."""
    R = np.asarray(R)
    b = np.arcsin(np.clip(R[0, 2], -1.0, 1.0))
    if abs(np.cos(b)) > 1e-9:
        a = np.arctan2(-R[1, 2], R[2, 2])
        c = np.arctan2(-R[0, 1], R[0, 0])
    else:  # gimbal lock: put everything in a
        a = np.arctan2(R[2, 1], R[1, 1])
        c = 0.0
    return np.array([a, b, c])


def rot_from_xyz(angles):
    a, b, c = angles
    ca, sa, cb, sb, cc, sc = np.cos(a), np.sin(a), np.cos(b), np.sin(b), np.cos(c), np.sin(c)
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    Rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("cannot normalise a zero vector")
    return v / n


def frame_from_y(y_axis, x_hint):
    """Right-handed rotation whose Y column is ``y_axis`` and X is as close as possible to ``x_hint``."""
    y = unit(y_axis)
    x = np.asarray(x_hint, dtype=float) - np.dot(x_hint, y) * y
    if np.linalg.norm(x) < 1e-9:
        x = np.cross(y, [0.0, 0.0, 1.0])
    x = unit(x)
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def offset_frame(name, parent, p_local, R_local=None):
    """PhysicalOffsetFrame on ``parent`` (not yet added to anything)."""
    frame = osim.PhysicalOffsetFrame()
    frame.setName(name)
    frame.setParentFrame(parent)
    frame.set_translation(vec3(p_local))
    if R_local is not None:
        frame.set_orientation(vec3(body_xyz_angles(R_local)))
    return frame


def complete_basis(axes):
    """Extend 1-3 (possibly non-orthogonal) unit axes to three independent axes.

    CustomJoint needs three independent rotation axes even when only one or
    two carry a coordinate.
    """
    out = [unit(a) for a in axes]
    for cand in np.eye(3):
        if len(out) == 3:
            break
        M = np.array(out + [cand])
        if np.linalg.matrix_rank(M, tol=1e-6) == len(out) + 1:
            # orthogonalise against the existing axes for numerical conditioning
            v = cand - sum(np.dot(cand, a) * a for a in out)
            out.append(unit(v) if np.linalg.norm(v) > 1e-6 else cand)
    return out


# ---------------------------------------------------------------- mass props
def rod_inertia(mass, length, radius, axis):
    """Solid cylinder about its centre, long axis ``axis`` (unit vector, body frame)."""
    a = unit(axis)
    i_ax = 0.5 * mass * radius ** 2
    i_perp = mass * (3 * radius ** 2 + length ** 2) / 12.0
    return i_perp * np.eye(3) + (i_ax - i_perp) * np.outer(a, a)


def ring_inertia(mass, radius, width, axis):
    """Thin cylindrical shell (cuff) about its centre."""
    a = unit(axis)
    i_ax = mass * radius ** 2
    i_perp = mass * (6 * radius ** 2 + width ** 2) / 12.0
    return i_perp * np.eye(3) + (i_ax - i_perp) * np.outer(a, a)


def box_inertia(mass, size, R=None):
    """Box with full edge lengths ``size`` (x, y, z), optionally rotated by R."""
    x, y, z = size
    I = mass / 12.0 * np.diag([y * y + z * z, x * x + z * z, x * x + y * y])
    return I if R is None else R @ I @ R.T


def combine(parts):
    """Combine [(mass, com, inertia_about_own_com)] -> (mass, com, inertia_about_com)."""
    m = sum(p[0] for p in parts)
    if m <= 0:
        raise ValueError("total mass must be positive")
    com = sum(p[0] * np.asarray(p[1]) for p in parts) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in parts:
        d = np.asarray(ci) - com
        I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, com, I


def osim_inertia(I):
    return osim.Inertia(float(I[0, 0]), float(I[1, 1]), float(I[2, 2]),
                        float(I[0, 1]), float(I[0, 2]), float(I[1, 2]))
