"""Landmarks of the human model needed to place the exoskeleton.

OpenSim invalidates the System as soon as a component is added, after which
no transform can be queried (this is why the legacy script re-ran initSystem()
~15 times). ``HumanLandmarks`` therefore takes a snapshot of everything it
needs at the default pose, in ground, *before* the model is modified, and all
methods below work on that snapshot.

Directions given in the config ("pelvis frame, right side") are mapped to
ground with ``dir_from_cfg`` / ``point_from_cfg`` so nothing assumes the model
faces +X.
"""
import numpy as np
import opensim as osim

from .geometry import to_np, transform_np, unit
from .markers import MarkerConventions

SIDE_SIGN = {"r": 1.0, "l": -1.0}


def _coordinate_axis(coord, state):
    """Ground direction about which a positive coordinate rotates at ``state``."""
    if coord.getMotionType() != 1:
        return None
    joint = coord.getJoint()
    R_parent, _ = transform_np(joint.getParentFrame(), state)
    custom = osim.CustomJoint.safeDownCast(joint)
    if custom is None:
        # Pin-like joints rotate about Z of the joint frames.
        return unit(R_parent[:, 2]) if joint.numCoordinates() == 1 else None
    st = custom.getSpatialTransform()
    best, best_slope = None, 0.0
    for i in range(3):  # rotation axes only
        ta = st.getTransformAxis(i)
        names = ta.getCoordinateNamesInArray()
        if coord.getName() not in [names.get(k) for k in range(names.getSize())]:
            continue
        f, h = ta.get_function(), 1e-3
        slope = (f.calcValue(osim.Vector(1, h)) - f.calcValue(osim.Vector(1, -h))) / (2 * h)
        if abs(slope) > abs(best_slope):
            best, best_slope = ta, slope
    if best is None:
        return None
    return unit(R_parent @ to_np(best.getAxis()) * np.sign(best_slope))


class HumanLandmarks:
    def __init__(self, model, cfg, state=None):
        self.model = model
        self.cfg = cfg
        state = state if state is not None else model.initSystem()
        self.markers = MarkerConventions(model) if cfg["human"].get("use_markers", True) else None

        self.body_pose = {b.getName(): transform_np(b, state) for b in model.getBodySet()}
        self.joint_center_g = {j.getName(): transform_np(j.getChildFrame(), state)[1]
                               for j in model.getJointSet()}
        self.coord_axis = {c.getName(): _coordinate_axis(c, state) for c in model.getCoordinateSet()}
        self.marker_g = {mk.getName(): to_np(mk.getLocationInGround(state)) for mk in model.getMarkerSet()}
        self.spheres = []
        cgs = model.getContactGeometrySet()
        for i in range(cgs.getSize()):
            sphere = osim.ContactSphere.safeDownCast(cgs.get(i))
            if sphere is None:
                continue
            R, p = transform_np(sphere.getFrame(), state)
            self.spheres.append({"name": sphere.getName(), "body": sphere.getFrame().getName(),
                                 "center": p + R @ to_np(sphere.get_location()),
                                 "radius": sphere.getRadius()})

        self.R0, self.p_pelvis = self.body_pose[self.body(cfg["human"]["pelvis"]).getName()]
        self.anterior = self.R0[:, 0]
        self.up = self.R0[:, 1]
        self.right = self.R0[:, 2]

    # ------------------------------------------------------------ lookups
    def body(self, name):
        bs = self.model.getBodySet()
        if not bs.contains(name):
            raise KeyError(f"human body '{name}' not found; bodies: "
                           f"{[bs.get(i).getName() for i in range(bs.getSize())]}")
        return bs.get(name)

    def has_coordinate(self, name):
        return bool(name) and self.coord_axis.get(name) is not None

    def coordinate_axis(self, name):
        axis = self.coord_axis.get(name)
        if axis is None:
            raise ValueError(f"cannot infer a rotation axis for coordinate {name!r}; give 'axis' explicitly")
        return axis

    def joint_center(self, joint_name):
        if joint_name not in self.joint_center_g:
            raise KeyError(f"human joint '{joint_name}' not found")
        return self.joint_center_g[joint_name]

    # ------------------------------------------------------- config frames
    def lateral(self, side):
        return SIDE_SIGN[side] * self.right

    def point_from_cfg(self, v, side=None):
        """Offset given in the pelvis frame (right-side convention) -> ground vector."""
        v = np.asarray(v, dtype=float)
        if side == "l":
            v = v * np.array([1.0, 1.0, -1.0])
        return self.R0 @ v

    def dir_from_cfg(self, axis, side):
        """Rotation axis (pseudo-vector) in pelvis frame, right side -> ground, mirrored for left."""
        a = np.asarray(axis, dtype=float)
        if side == "l":
            a = a * np.array([-1.0, -1.0, 1.0])
        return unit(self.R0 @ a)

    # ----------------------------------------------------------- landmarks
    def segment_ends(self, view, segment):
        """Proximal and distal joint centres (ground) of a human thigh/shank."""
        j = view["human"]["joints"]
        if segment == "thigh":
            return self.joint_center(j["hip"]), self.joint_center(j["knee"])
        if segment == "shank":
            return self.joint_center(j["knee"]), self.joint_center(j["ankle"])
        raise ValueError(segment)

    def _marker_ml_distance(self, side, key, center):
        if self.markers is None:
            return None
        name = self.markers.side(side, key)
        if name is None:
            return None
        d = abs(np.dot(self.marker_g[name] - center, self.right))
        return d if 0.02 < d < 0.15 else None

    def limb_radius(self, view, side, segment, at):
        """Limb radius (m) at fraction ``at`` along the segment."""
        prox, dist = self.segment_ends(view, segment)
        length = np.linalg.norm(dist - prox)
        r_prox, r_dist = [length * k for k in view["human"]["radius_ratio"][segment]]
        if segment == "thigh":
            r_dist = self._marker_ml_distance(side, "knee", dist) or r_dist
        elif segment == "shank":
            r_prox = self._marker_ml_distance(side, "knee", prox) or r_prox
            r_dist = self._marker_ml_distance(side, "ankle", dist) or r_dist
        return (1 - at) * r_prox + at * r_dist

    def foot_contacts(self, view):
        """Contact spheres on the human foot/toes bodies of this side."""
        bodies = {view["human"]["bodies"]["foot"], view["human"]["bodies"].get("toes")}
        return [s for s in self.spheres if s["body"] in bodies]

    def foot_footprint(self, view, side):
        """Sole box of the human foot (ground): centre on the sole, length/width, sole height."""
        spheres = self.foot_contacts(view)
        if spheres:
            sole = min(np.dot(s["center"], self.up) - s["radius"] for s in spheres)
            r = max(s["radius"] for s in spheres)
            xs = [np.dot(s["center"], self.anterior) for s in spheres]
            zs = [np.dot(s["center"], self.right) for s in spheres]
            x0, x1, z0, z1 = min(xs) - r, max(xs) + r, min(zs) - r, max(zs) + r
        else:
            ankle = self.joint_center(view["human"]["joints"]["ankle"])
            _, pf = self.body_pose[view["human"]["bodies"]["foot"]]
            sole = np.dot(pf, self.up) - 0.02
            xa, za = np.dot(ankle, self.anterior), np.dot(ankle, self.right)
            x0, x1, z0, z1 = xa - 0.07, xa + 0.19, za - 0.045, za + 0.045
        center = self.anterior * (x0 + x1) / 2 + self.right * (z0 + z1) / 2 + self.up * sole
        return {"center": center, "length": x1 - x0, "width": z1 - z0, "sole": sole}
