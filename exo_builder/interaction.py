"""Human <-> exo interaction elements: cuffs, belt, foot binding and foot contact.

Every coupling is a pair of PhysicalOffsetFrames that coincide at the default
pose (zero preload): ``<name>_human`` on the human body (frame1) and
``<name>_exo`` on the exo body (frame2). Frame axes: X = anterior (ap),
Y = along the segment / up (longitudinal), Z = X x Y (mediolateral, points right).

Notes from probing the OpenSim 4.5/4.6 Python API (see tests/test_api_contract.py):
* BushingForce has no calcStiffnessForce/calcDampingForce in Python; read the
  forces with getRecordValues() (see analysis.py).
* ExpressionBasedBushingForce expressions only know theta_x..z / delta_x..z.
  A positive expression value is a restoring force (same sign as BushingForce).
"""
import numpy as np
import opensim as osim

from .geometry import offset_frame, vec3

AXES = ("anteroposterior", "longitudinal", "mediolateral")  # coupling frame X, Y, Z
SSHSF_PROPS = ("stiffness", "dissipation", "static_friction", "dynamic_friction", "viscous_friction",
               "transition_velocity", "constant_contact_force", "hertz_smoothing", "hunt_crossley_smoothing")


def _v(d):
    return osim.Vec3(*[float(d[k]) for k in AXES])


def coupling_frames(name, center_g, R_g, human_body, exo_body, human_pose, exo_pose):
    """Frames on both bodies, coincident with (center_g, R_g) at the default pose."""
    R_h, p_h = human_pose
    R_e, p_e = exo_pose
    f_h = offset_frame(f"{name}_human", human_body, R_h.T @ (center_g - p_h), R_h.T @ R_g)
    f_e = offset_frame(f"{name}_exo", exo_body, R_e.T @ (center_g - p_e), R_e.T @ R_g)
    human_body.addComponent(f_h)
    exo_body.addComponent(f_e)
    return f_h, f_e


def make_coupling(name, f_h, f_e, spec):
    """Force/constraint linking the two frames according to ``spec['model']``."""
    model = spec.get("model", "linear")
    if model == "weld":
        wc = osim.WeldConstraint()
        wc.setName(name)
        wc.connectSocket_frame1(f_h)
        wc.connectSocket_frame2(f_e)
        return wc
    if model in ("linear", "bushing"):
        return osim.BushingForce(name, f_h, f_e,
                                 _v(spec["stiffness_translational"]), _v(spec["stiffness_rotational"]),
                                 _v(spec["damping_translational"]), _v(spec["damping_rotational"]))
    if model == "nonlinear":
        f = osim.ExpressionBasedBushingForce(name, f_h, f_e)
        cubic = spec.get("stiffness_cubic", {})
        for ax, key, var in zip(AXES, ("Fx", "Fy", "Fz"), ("delta_x", "delta_y", "delta_z")):
            k1, k3 = float(spec["stiffness_translational"][ax]), float(cubic.get(ax, 0.0))
            expr = f"{k1}*{var}" + (f" + {k3}*{var}^3" if k3 else "")
            getattr(f, f"set{key}Expression")(expr)
        for ax, key, var in zip(AXES, ("Mx", "My", "Mz"), ("theta_x", "theta_y", "theta_z")):
            getattr(f, f"set{key}Expression")(f"{float(spec['stiffness_rotational'][ax])}*{var}")
        for key, expr in (spec.get("expressions") or {}).items():
            getattr(f, f"set{key}Expression")(str(expr))
        f.set_translational_damping(_v(spec["damping_translational"]))
        f.set_rotational_damping(_v(spec["damping_rotational"]))
        return f
    raise ValueError(f"unknown coupling model {model!r}")


def add_coupling(builder, name, kind, side, segment, human_body, exo_body, center_g, R_g, spec):
    """Create frames + force/constraint and register the coupling in ``builder.info``."""
    m = builder.model
    exo_pose = builder.exo_pose[exo_body.getName()]
    f_h, f_e = coupling_frames(name, center_g, R_g, human_body, exo_body,
                               builder.L.body_pose[human_body.getName()], exo_pose)
    elem = make_coupling(name, f_h, f_e, spec)
    if isinstance(elem, osim.Constraint):
        m.addConstraint(elem)
    else:
        m.addForce(elem)
    builder.info["couplings"].append({
        "name": name, "kind": kind, "side": side, "segment": segment,
        "model": spec.get("model", "linear"),
        "human_body": human_body.getName(), "exo_body": exo_body.getName(),
        "center": [float(x) for x in center_g]})
    return f_h, f_e


# ------------------------------------------------------------- foot contact
def add_plate_contact(builder, view, side, exo_foot, plate_top_center_g, thickness):
    """plate_contact mode: human spheres push on the plate, plate spheres push on the floor."""
    m, L = builder.model, builder.L
    R_e, p_e = builder.exo_pose[exo_foot.getName()]
    cgs = m.getContactGeometrySet()
    floor_name = view["human"]["floor"]
    if not cgs.contains(floor_name):
        raise KeyError(f"floor contact geometry '{floor_name}' not found")
    floor = osim.ContactHalfSpace.safeDownCast(cgs.get(floor_name))  # owned by the model: safe

    # Half-space on top of the plate. ContactHalfSpace's outward normal is -X of
    # its frame, so rotate -90 deg about Z to point it up (like the floor).
    hs_name = f"exo_foot_{side}_plate"
    hs = osim.ContactHalfSpace(vec3(R_e.T @ (plate_top_center_g - p_e)), osim.Vec3(0, 0, -np.pi / 2),
                               exo_foot, hs_name)
    m.addContactGeometry(hs)

    spheres = {s["name"]: s for s in L.foot_contacts(view)}
    sole = L.foot_footprint(view, side)["sole"]
    moved, added = [], []
    fs = m.getForceSet()
    human_forces = []
    for i in range(fs.getSize()):
        f = osim.SmoothSphereHalfSpaceForce.safeDownCast(fs.get(i))
        if f is None:
            continue
        sphere_path = f.getSocket("sphere").getConnecteePath()
        sname = sphere_path.split("/")[-1]
        if sname in spheres:
            human_forces.append((f, sname))
    if not human_forces:
        raise RuntimeError(f"plate_contact: no SmoothSphereHalfSpaceForce found on the {side} foot")
    for f, sname in human_forces:
        s = spheres[sname]
        f.connectSocket_half_space(hs)
        moved.append(f.getName())
        # Copy of the sphere under the plate, touching the plate bottom.
        c = s["center"] + (sole - thickness + s["radius"] - np.dot(s["center"], L.up)) * L.up
        exo_sphere = osim.ContactSphere(s["radius"], vec3(R_e.T @ (c - p_e)), exo_foot, f"exo_{sname}")
        m.addContactGeometry(exo_sphere)
        # Do not use safeDownCast(f.clone()): the clone is a Python temporary that
        # gets freed, and the next call aborts the interpreter.
        f2 = osim.SmoothSphereHalfSpaceForce(f"exo_{f.getName()}", exo_sphere, floor)
        for prop in SSHSF_PROPS:
            getattr(f2, f"set_{prop}")(getattr(f, f"get_{prop}")())
        m.addForce(f2)
        added.append(f2.getName())
    builder.info["contacts"].append({"side": side, "plate": hs_name, "human_forces_on_plate": moved,
                                     "plate_forces_on_floor": added})
