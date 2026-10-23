"""Build an exoskeleton around a human OpenSim model from a configuration.

Kinematic layout (per side, every part optional):

    human pelvis --(weld | 6-DOF root + belt bushing)--> exo_pelvis
    exo_pelvis --exo_hip_s--> exo_thigh_s --exo_knee_s--> exo_shank_s --exo_ankle_s--> exo_foot_s

A segment whose parent is disabled becomes a floating root (6-DOF joint to
ground) held to the human only by its cuffs, which is what allows knee-only or
ankle-only exos. Floating roots are CustomJoints (rotations Z-X-Y, then
translations, like Rajagopal's ground_pelvis), not FreeJoints: Moco rejects
FreeJoints because their speeds are not the coordinate derivatives. Exo joints sit at the human joint centres (+ misalignment) and,
by default, rotate about the same axes as the matching human coordinates, so a
positive exo angle means the same motion as the positive human angle.

Every exo body has the pelvis orientation at the default pose, its origin at
its proximal joint centre; positions are computed once, analytically, from the
human landmarks (no repeated initSystem calls).
"""
from collections import Counter
from pathlib import Path

import numpy as np
import opensim as osim

from . import config as C
from .geometry import (body_xyz_angles, box_inertia, combine, complete_basis, frame_from_y,
                       offset_frame, osim_inertia, ring_inertia, rod_inertia, unit, vec3)
from .human import HumanLandmarks
from .interaction import add_coupling, add_plate_contact

# 6-DOF root: (coordinate suffix, axis in the ground-side frame, rotational?)
ROOT_DOFS = (("rz", (0, 0, 1), True), ("rx", (1, 0, 0), True), ("ry", (0, 1, 0), True),
             ("tx", (1, 0, 0), False), ("ty", (0, 1, 0), False), ("tz", (0, 0, 1), False))
PROXIMAL_JOINT = {"thigh": "hip", "shank": "knee", "foot": "ankle"}
DISTAL_JOINT = {"thigh": "knee", "shank": "ankle"}


class ExoBuilder:
    def __init__(self, model, cfg=None):
        self.model = osim.Model(str(model)) if isinstance(model, (str, Path)) else model
        self.cfg = C.load_config(cfg)
        self.info = {"name": self.cfg["name"], "bodies": [], "joints": [], "coordinates": [],
                     "actuators": [], "couplings": [], "contacts": [], "warnings": []}
        self.exo_pose = {}  # body name -> (R, origin) in ground at the default pose
        self.colors = {}

    # ------------------------------------------------------------------ API
    def build(self):
        # Snapshot of the human at its default pose; the System is invalid from here on.
        self.L = HumanLandmarks(self.model, self.cfg)
        self.R0 = self.L.R0
        if self.cfg["pelvis_unit"]["enabled"]:
            self._add_pelvis_unit()
        for side in self.cfg["sides"]:
            self._add_side(side)
        self.model.finalizeConnections()
        self._check_unique_names()
        self.info["exo_mass"] = float(sum(self.model.getBodySet().get(b).getMass()
                                          for b in self.info["bodies"]))
        return self.model

    # ------------------------------------------------------------- helpers
    def _local(self, body_name, p_g):
        R, o = self.exo_pose[body_name]
        return R.T @ (np.asarray(p_g) - o)

    def _new_body(self, name, origin_g, parts_g, color):
        """Body whose mass properties combine ``parts_g`` = [(mass, com_ground, inertia_ground)]."""
        self.exo_pose[name] = (self.R0, np.asarray(origin_g, dtype=float))
        mass, com, inertia = combine([(m, self._local(name, c), self.R0.T @ I @ self.R0)
                                      for m, c, I in parts_g])
        body = osim.Body(name, mass, vec3(com), osim_inertia(inertia))
        self.info["bodies"].append(name)
        self.colors[name] = color
        return body

    def _geom(self, body, name, shape, p_g, R_g=None, color=(1, 1, 0), opacity=1.0):
        """Attach visual geometry through an offset frame placed in ground coordinates."""
        R_local = None if R_g is None else self.R0.T @ R_g
        frame = offset_frame(f"{body.getName()}_{name}_geom", body, self._local(body.getName(), p_g), R_local)
        body.addComponent(frame)
        shape.setColor(vec3(color))
        shape.setOpacity(float(opacity))
        frame.attachGeometry(shape)
        return frame

    def _add_free_root(self, body, origin_g):
        """6-DOF CustomJoint from ground (Moco compatible, unlike FreeJoint)."""
        name = f"{body.getName()}_free"
        prefix = body.getName()
        st = osim.SpatialTransform()
        for i, (suffix, axis, _) in enumerate(ROOT_DOFS):
            ta = st.getTransformAxis(i)
            ta.set_coordinates(0, f"{prefix}_{suffix}")
            ta.setAxis(osim.Vec3(*axis))
            ta.set_function(osim.LinearFunction(1, 0))
        j = osim.CustomJoint(name, self.model.getGround(), vec3(origin_g), vec3(body_xyz_angles(self.R0)),
                             body, osim.Vec3(0), osim.Vec3(0), st)
        self.model.addJoint(j)  # CustomJoint coordinates only exist after addJoint()
        for i, (_, _, rotational) in enumerate(ROOT_DOFS):
            c = j.upd_coordinates(i)
            lim = np.pi if rotational else 5.0
            c.set_range(0, -lim)
            c.set_range(1, lim)
            c.setDefaultClamped(False)
            self.info["coordinates"].append(c.getName())
        self.info["joints"].append(name)
        return j

    def _segment_frame(self, prox_g, dist_g):
        """Cuff/segment frame: Y from distal to proximal, X anterior."""
        return frame_from_y(prox_g - dist_g, self.L.anterior)

    # ---------------------------------------------------------- pelvis unit
    def _add_pelvis_unit(self):
        cfg, L = self.cfg, self.L
        pu = cfg["pelvis_unit"]
        color = cfg["appearance"]["color_pelvis"]
        origin = L.p_pelvis + L.point_from_cfg(pu["belt"]["offset"])
        # Belt half-width: hip centre + thigh radius + clearance (largest of both sides).
        half_w = 0.0
        for side in ("r", "l"):
            view = C.side_view(cfg, side)
            hjc = L.joint_center(view["human"]["joints"]["hip"])
            r_thigh = L.limb_radius(view, side, "thigh", 0.0)
            half_w = max(half_w, abs(np.dot(hjc - origin, L.right)) + r_thigh)
        half_w += pu["belt"]["clearance"]
        half_d = 0.75 * half_w
        self.belt_half_width = half_w

        parts = [(pu["mass"], origin, ring_inertia(pu["mass"], (half_w + half_d) / 2, pu["belt"]["height"], L.up))]
        bp = pu["backpack"]
        bp_center = origin + L.point_from_cfg(bp["offset"])
        if bp.get("enabled", True) and bp["mass"] > 0:
            parts.append((bp["mass"], bp_center, box_inertia(bp["mass"], bp["size"], self.R0)))
        body = self._new_body("exo_pelvis", origin, parts, color)
        self.model.addBody(body)

        self._geom(body, "belt", osim.Ellipsoid(half_d, pu["belt"]["height"] / 2, half_w), origin,
                   color=color, opacity=0.5)
        if bp.get("enabled", True):
            self._geom(body, "backpack", osim.Brick(vec3(np.asarray(bp["size"]) / 2)), bp_center, self.R0, color)

        pelvis = L.body(cfg["human"]["pelvis"])
        attach = pu["attach"]
        if attach.get("model") == "weld":
            R_p = L.R0
            j = osim.WeldJoint("exo_pelvis_attach", pelvis, vec3(R_p.T @ (origin - L.p_pelvis)),
                               vec3(body_xyz_angles(R_p.T @ self.R0)), body, osim.Vec3(0), osim.Vec3(0))
            self.model.addJoint(j)
            self.info["joints"].append(j.getName())
        else:
            self._add_free_root(body, origin)
            add_coupling(self, "belt", "belt", None, "pelvis", pelvis, body, origin, self.R0, attach)

    # ---------------------------------------------------------------- sides
    def _add_side(self, side):
        cfg, L = self.cfg, self.L
        view = C.side_view(cfg, side)
        color = cfg["appearance"][f"color_{side}"]
        lat = L.lateral(side)
        human_jc = {j: L.joint_center(view["human"]["joints"][j]) for j in ("hip", "knee", "ankle")}
        exo_jc = {j: human_jc[j] + L.point_from_cfg(view["joints"][j]["misalignment"], side)
                  for j in human_jc}
        bodies = {"pelvis": self.model.getBodySet().get("exo_pelvis") if cfg["pelvis_unit"]["enabled"] else None}

        for seg in C.SEGMENTS:
            spec = view["segments"][seg]
            if not spec.get("enabled", True):
                bodies[seg] = None
                continue
            name = f"exo_{seg}_{side}"
            origin = exo_jc[PROXIMAL_JOINT[seg]]
            if seg == "foot":
                body, extra = self._foot_body(view, side, name, origin, color)
            else:
                body, extra = self._limb_body(view, side, seg, name, origin, exo_jc, human_jc, color, lat)
            self.model.addBody(body)
            for g in extra:
                self._geom(body, **g)
            parent = bodies[C.PARENT_OF[seg]]
            if parent is not None:
                self._add_joint(view, side, PROXIMAL_JOINT[seg], parent, body, origin, lat)
            else:
                self._add_free_root(body, origin)
                self.info["warnings"].append(
                    f"{name} is a floating root (parent '{C.PARENT_OF[seg]}' disabled): it is held "
                    f"only by its cuffs and its 6 root coordinates need initial values/tracking")
            bodies[seg] = body
            human_body = L.body(view["human"]["bodies"][seg])
            if seg == "foot":
                self._foot_interaction(view, side, body, human_body)
            else:
                self._cuffs(view, side, seg, body, human_body, human_jc)
        self._check_weld_loop(view, side, bodies)

    def _bar_offset(self, view, side, seg):
        spec = view["segments"][seg]
        r = max(self.L.limb_radius(view, side, seg, a) for a in (0.0, 1.0))
        return r + spec["clearance"] + spec["bar_radius"]

    def _limb_body(self, view, side, seg, name, origin, exo_jc, human_jc, color, lat):
        spec = view["segments"][seg]
        dist = exo_jc[DISTAL_JOINT[seg]]
        off = self._bar_offset(view, side, seg)
        bar_c = (origin + dist) / 2 + lat * off
        length = np.linalg.norm(origin - dist)
        axis = unit(origin - dist)
        cuffs = [C.cuff_spec(view, c) for c in spec.get("cuffs", [])]
        cuff_mass = min(0.2 * len(cuffs), 0.6) * spec["mass"]
        parts = [(spec["mass"] - cuff_mass, bar_c, rod_inertia(spec["mass"] - cuff_mass, length, spec["bar_radius"], axis))]
        h_prox, h_dist = human_jc[PROXIMAL_JOINT[seg]], human_jc[DISTAL_JOINT[seg]]
        for c in cuffs:
            center = h_prox + c["along_segment"] * (h_dist - h_prox)
            r = self.L.limb_radius(view, side, seg, c["along_segment"]) if c["radius"] == "auto" else c["radius"]
            parts.append((cuff_mass / len(cuffs), center, ring_inertia(cuff_mass / len(cuffs), r, c["width"], axis)))
        body = self._new_body(name, origin, parts, color)
        R_bar = frame_from_y(axis, self.L.anterior)
        geoms = [dict(name="bar", shape=osim.Cylinder(spec["bar_radius"], length / 2), p_g=bar_c, R_g=R_bar,
                      color=color)]
        # Joint disc at the proximal joint, on the bar, around the first DOF axis.
        joint = PROXIMAL_JOINT[seg]
        dofs = [d for d in view["joints"][joint]["dofs"].values() if C.dof_spec(d)["enabled"]]
        if dofs:
            ax = self._dof_axis(view, side, joint, next(k for k, d in view["joints"][joint]["dofs"].items()
                                                        if C.dof_spec(d)["enabled"]))
            geoms.append(dict(name="joint", shape=osim.Cylinder(0.03, 0.012), p_g=origin + lat * off,
                              R_g=frame_from_y(ax, self.L.anterior if abs(np.dot(ax, self.L.anterior)) < 0.9 else self.L.up),
                              color=color))
        return body, geoms

    def _foot_body(self, view, side, name, origin, color):
        spec = view["segments"]["foot"]
        fp = self.L.foot_footprint(view, side)
        plate = spec["plate"]
        t = plate["thickness"]
        length = fp["length"] + 2 * plate["margin"] if plate["length"] == "auto" else plate["length"]
        width = fp["width"] + 2 * plate["margin"] if plate["width"] == "auto" else plate["width"]
        center = fp["center"] - self.L.up * t / 2
        self._plate = {"top": fp["center"], "thickness": t}
        body = self._new_body(name, origin, [(spec["mass"], center,
                                              box_inertia(spec["mass"], [length, t, width], self.R0))], color)
        lat = self.L.lateral(side)
        strut_top = origin + lat * (self._bar_offset(view, side, "shank") if view["segments"]["shank"].get("enabled", True)
                                    else fp["width"] / 2 + 0.02)
        strut_bot = strut_top + self.L.up * (fp["sole"] - np.dot(strut_top, self.L.up))
        geoms = [dict(name="plate", shape=osim.Brick(osim.Vec3(length / 2, t / 2, width / 2)), p_g=center,
                      R_g=self.R0, color=color),
                 dict(name="strut", shape=osim.Cylinder(0.008, np.linalg.norm(strut_top - strut_bot) / 2),
                      p_g=(strut_top + strut_bot) / 2, R_g=self.R0, color=color)]
        return body, geoms

    # --------------------------------------------------------------- joints
    def _dof_axis(self, view, side, joint, dof_name):
        d = C.dof_spec(view["joints"][joint]["dofs"][dof_name])
        if d["axis"] == "human":
            if self.L.has_coordinate(d["human_coord"]):
                return self.L.coordinate_axis(d["human_coord"])
            fallback = C.FALLBACK_AXES.get(dof_name)
            if fallback is None:
                raise C.ConfigError(f"joints.{joint}.dofs.{dof_name}: human coordinate "
                                    f"{d['human_coord']!r} not in the model; give 'axis' explicitly")
            return self.L.dir_from_cfg(fallback, side)
        return self.L.dir_from_cfg(d["axis"], side)

    def _add_joint(self, view, side, joint, parent, child, center_g, lat):
        jname = f"exo_{joint}_{side}"
        p_parent = vec3(self._local(parent.getName(), center_g))
        dofs = [(k, C.dof_spec(d)) for k, d in view["joints"][joint]["dofs"].items() if C.dof_spec(d)["enabled"]]
        if not dofs:
            j = osim.WeldJoint(jname, parent, p_parent, osim.Vec3(0), child, osim.Vec3(0), osim.Vec3(0))
            self.model.addJoint(j)
            self.info["joints"].append(jname)
            return j
        if len(dofs) > 3:
            raise C.ConfigError(f"joints.{joint}: at most 3 rotational DOFs")
        axes = [self.R0.T @ self._dof_axis(view, side, joint, k) for k, _ in dofs]
        st = osim.SpatialTransform()  # a fresh one per joint (CustomJoint copies it anyway)
        for i, a in enumerate(complete_basis(axes)):
            ta = st.getTransformAxis(i)
            ta.setAxis(vec3(a))
            if i < len(dofs):
                ta.set_coordinates(0, f"exo_{joint}_{dofs[i][0]}_{side}")
                ta.set_function(osim.LinearFunction(1, 0))
        j = osim.CustomJoint(jname, parent, p_parent, osim.Vec3(0), child, osim.Vec3(0), osim.Vec3(0), st)
        self.model.addJoint(j)
        self.info["joints"].append(jname)
        for i, (dname, d) in enumerate(dofs):
            coord = j.upd_coordinates(i)
            lo, hi = np.radians(d["range"])
            coord.set_range(0, float(lo))
            coord.set_range(1, float(hi))
            coord.setDefaultValue(float(np.radians(d["default"])))
            coord.setDefaultClamped(d["limits"] == "clamp")
            cname = coord.getName()
            self.info["coordinates"].append(cname)
            self._dof_forces(cname, d)
        return j

    def _dof_forces(self, cname, d):
        if d["actuated"]:
            act = osim.CoordinateActuator(cname)
            act.setName(f"tau_{cname}")
            act.setOptimalForce(float(d["max_torque"]))
            act.setMinControl(-1.0)
            act.setMaxControl(1.0)
            self.model.addForce(act)
            self.info["actuators"].append(act.getName())
        if d["passive_stiffness"] > 0 or d["passive_damping"] > 0:
            spring = osim.SpringGeneralizedForce(cname)
            spring.setName(f"passive_{cname}")
            spring.setStiffness(float(d["passive_stiffness"]))
            spring.setViscosity(float(d["passive_damping"]))
            spring.setRestLength(float(np.radians(d["default"])))
            self.model.addForce(spring)
        if d["limits"] == "soft":
            lo, hi = d["range"]  # CoordinateLimitForce works in degrees for rotations
            clf = osim.CoordinateLimitForce(cname, float(hi), float(d["limit_stiffness"]), float(lo),
                                            float(d["limit_stiffness"]), float(d["limit_damping"]), 2.0)
            clf.setName(f"limit_{cname}")
            self.model.addForce(clf)

    # -------------------------------------------------------- interaction
    def _cuffs(self, view, side, seg, body, human_body, human_jc):
        prox, dist = human_jc[PROXIMAL_JOINT[seg]], human_jc[DISTAL_JOINT[seg]]
        R_seg = self._segment_frame(prox, dist)
        opacity = self.cfg["appearance"]["cuff_opacity"]
        for i, raw in enumerate(view["segments"][seg].get("cuffs", [])):
            c = C.cuff_spec(view, raw)
            name = f"{c.get('name') or f'{seg}_cuff{i}'}_{side}"
            center = prox + c["along_segment"] * (dist - prox) + R_seg @ np.asarray(c["offset"], dtype=float)
            add_coupling(self, name, "cuff", side, seg, human_body, body, center, R_seg, c)
            r = self.L.limb_radius(view, side, seg, c["along_segment"]) if c["radius"] == "auto" else c["radius"]
            self._geom(body, f"{name}_shell", osim.Cylinder(r + 0.004, c["width"] / 2), center, R_seg,
                       self.colors[body.getName()], opacity)
            self.info["couplings"][-1].update({"along_segment": c["along_segment"], "radius": float(r), "width": c["width"]})

    def _foot_interaction(self, view, side, body, human_body):
        spec = view["segments"]["foot"]
        top = self._plate["top"]
        binding = dict(spec["binding"])
        if spec["mode"] == "weld":
            binding["model"] = "weld"
        add_coupling(self, f"binding_{side}", "binding", side, "foot", human_body, body, top, self.R0, binding)
        if spec["mode"] == "plate_contact":
            add_plate_contact(self, view, side, body, top, self._plate["thickness"])

    # ---------------------------------------------------------- checks
    def _check_weld_loop(self, view, side, bodies):
        foot = view["segments"]["foot"]
        if not (bodies.get("foot") and foot["mode"] == "weld"):
            return
        n = 0
        for seg in ("foot", "shank", "thigh"):
            if bodies.get(seg) is None:
                n += 6  # floating root: free
                break
            n += sum(1 for d in view["joints"][PROXIMAL_JOINT[seg]]["dofs"].values() if C.dof_spec(d)["enabled"])
        else:
            n += 6 if self.cfg["pelvis_unit"]["attach"].get("model") != "weld" else 0
        if n < 6:
            self.info["warnings"].append(
                f"side {side}: foot weld closes a loop through an exo chain with only {n} DOFs (< 6); "
                f"the human leg will be over-constrained. Use foot mode 'bushing' or add DOFs.")

    def _check_unique_names(self):
        m = self.model
        groups = {
            "coordinate": [c.getName() for c in m.getCoordinateSet()],
            "body": [b.getName() for b in m.getBodySet()],
            "joint": [j.getName() for j in m.getJointSet()],
            "force": [f.getName() for f in m.getForceSet()],
            "constraint": [c.getName() for c in m.getConstraintSet()],
            "contact geometry": [g.getName() for g in m.getContactGeometrySet()],
        }
        # OpenSim accepts duplicate names silently (e.g. the legacy q_rz/q_ry), so check here.
        dups = {k: [n for n, c in Counter(v).items() if c > 1] for k, v in groups.items()}
        dups = {k: v for k, v in dups.items() if v}
        if dups:
            raise RuntimeError(f"duplicate component names: {dups}")


def build_exo(model, cfg=None):
    """Return (model_with_exo, info). ``model``: path or osim.Model; ``cfg``: path, preset name or dict."""
    b = ExoBuilder(model, cfg)
    return b.build(), b.info
