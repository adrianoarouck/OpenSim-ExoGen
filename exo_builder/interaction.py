"""Human <-> exo interaction elements: cuffs, belt, foot binding and foot contact.

Every coupling is a pair of PhysicalOffsetFrames that coincide at the default
pose (zero preload): ``<name>_human`` on the human body (frame1) and
``<name>_exo`` on the exo body (frame2). Frame axes: X = anterior (ap),
Y = along the segment / up (longitudinal), Z = X x Y (mediolateral, points right).

Cuff models (``config.MODEL_DOCS`` has the explanation and references):

* ``linear`` / ``nonlinear``: one bushing ``<name>`` between the two frames.
* ``ring``: body ``exo_<name>_ring`` on a 6-DOF joint from ``<name>_human``;
  tissue bushing ``<name>`` (human -> ring) in series with the strap bushing
  ``<name>_strap`` (ring -> exo).
* ``sliding``: the ring slides (``<name>_slide``) and twists (``<name>_twist``)
  on the limb axis against smooth Coulomb friction; tissue bushing ``<name>``
  (ring -> exo), so the force point is fixed on the exo and moves on the limb.
* ``contact``: spheres ``<name>_pad<row>_<k>`` on the exo press on half-spaces
  tangent to the skin (SmoothSphereHalfSpaceForce, compression only).

analysis.py and kinematics.py find these elements by the names above.

Notes from probing the OpenSim 4.5/4.6 Python API (see tests/test_api_contract.py):
* BushingForce has no calcStiffnessForce/calcDampingForce in Python; read the
  forces with getRecordValues() (see analysis.py).
* ExpressionBasedBushingForce expressions only know theta_x..z / delta_x..z.
  A positive expression value is a restoring force (same sign as BushingForce).
"""
import functools
import re

import numpy as np
import opensim as osim

from .geometry import body_xyz_angles, offset_frame, osim_inertia, ring_inertia, vec3

AXES = ("anteroposterior", "longitudinal", "mediolateral")  # coupling frame X, Y, Z
SSHSF_PROPS = ("stiffness", "dissipation", "static_friction", "dynamic_friction", "viscous_friction",
               "transition_velocity", "constant_contact_force", "hertz_smoothing", "hunt_crossley_smoothing")


RING_MODELS = ("ring", "sliding")
STRAP_SUFFIX = "_strap"
SLIDING_SUFFIXES = ("_slide", "_twist")  # sliding-cuff coordinates: friction-held, not fitted
PAD_RE = re.compile(r"^(?P<cuff>.+)_pad\d+_\d+$")


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


def _register(builder, name, kind, side, segment, human_body, exo_body, center_g, spec):
    builder.info["couplings"].append({
        "name": name, "kind": kind, "side": side, "segment": segment,
        "model": spec.get("model", "linear"),
        "human_body": human_body.getName(), "exo_body": exo_body.getName(),
        "center": [float(x) for x in center_g]})


def _frames(builder, name, human_body, exo_body, center_g, R_g):
    return coupling_frames(name, center_g, R_g, human_body, exo_body,
                           builder.L.body_pose[human_body.getName()], builder.exo_pose[exo_body.getName()])


def add_coupling(builder, name, kind, side, segment, human_body, exo_body, center_g, R_g, spec):
    """Create frames + force/constraint and register the coupling in ``builder.info``."""
    m = builder.model
    f_h, f_e = _frames(builder, name, human_body, exo_body, center_g, R_g)
    elem = make_coupling(name, f_h, f_e, spec)
    if isinstance(elem, osim.Constraint):
        m.addConstraint(elem)
    else:
        m.addForce(elem)
    _register(builder, name, kind, side, segment, human_body, exo_body, center_g, spec)
    return f_h, f_e


# ------------------------------------------------------------------- cuffs
def _tissue_spec(spec):
    """Soft-tissue bushing of the ring/sliding models: cubic terms make it nonlinear."""
    nonlinear = any(spec.get("stiffness_cubic", {}).values()) or spec.get("expressions")
    return dict(spec, model="nonlinear" if nonlinear else "linear")


def add_cuff(builder, name, side, segment, human_body, exo_body, center_g, R_g, spec, radius, ring_mass):
    """Cuff with the model chosen in ``spec['model']``. Returns the body that carries the cuff shell."""
    model = spec.get("model", "linear")
    if model not in RING_MODELS + ("contact",):
        add_coupling(builder, name, "cuff", side, segment, human_body, exo_body, center_g, R_g, spec)
        return exo_body
    m = builder.model
    f_h, f_e = _frames(builder, name, human_body, exo_body, center_g, R_g)
    shell_body = exo_body
    if model in RING_MODELS:
        mass = ring_mass if spec["ring"]["mass"] == "auto" else float(spec["ring"]["mass"])
        ring = _ring_body(builder, name, human_body, f_h, mass, radius, spec["width"], sliding=model == "sliding")
        if model == "ring":
            m.addForce(make_coupling(name, f_h, ring, _tissue_spec(spec)))
            m.addForce(make_coupling(name + STRAP_SUFFIX, ring, f_e, dict(spec["ring"]["strap"], model="linear")))
        else:
            m.addForce(make_coupling(name, ring, f_e, _tissue_spec(spec)))
            _add_friction(m, name, spec["sliding"], radius)
        shell_body = ring
    else:
        _add_contact_pads(builder, name, side, human_body, exo_body, center_g, R_g, spec, radius)
    _register(builder, name, "cuff", side, segment, human_body, exo_body, center_g, spec)
    return shell_body


def _ring_body(builder, name, human_body, f_h, mass, radius, width, sliding):
    """Cuff ring on a joint whose parent frame is ``<name>_human`` (ring frame = cuff frame at rest)."""
    m = builder.model
    body = osim.Body(f"exo_{name}_ring", mass, osim.Vec3(0),
                     osim_inertia(ring_inertia(mass, radius, width, np.array([0.0, 1.0, 0.0]))))
    m.addBody(body)
    builder.info["bodies"].append(body.getName())
    st = osim.SpatialTransform()
    if sliding:
        # rotation about the limb axis (Y) + translation along it; other axes frozen
        dofs = {0: (f"{name}_twist", (0, 1, 0)), 4: (f"{name}_slide", (0, 1, 0))}
        axes = {1: (1, 0, 0), 2: (0, 0, 1), 3: (1, 0, 0), 5: (0, 0, 1)}
    else:
        dofs = {i: (f"exo_{name}_ring_{sfx}", ax) for i, (sfx, ax) in
                enumerate((("rz", (0, 0, 1)), ("rx", (1, 0, 0)), ("ry", (0, 1, 0)),
                           ("tx", (1, 0, 0)), ("ty", (0, 1, 0)), ("tz", (0, 0, 1))))}
        axes = {}
    for i in range(6):
        ta = st.getTransformAxis(i)
        if i in dofs:
            ta.set_coordinates(0, dofs[i][0])
            ta.setAxis(osim.Vec3(*dofs[i][1]))
            ta.set_function(osim.LinearFunction(1, 0))
        else:
            ta.setAxis(osim.Vec3(*axes[i]))
    j = osim.CustomJoint(f"{name}_ring_joint", human_body, f_h.get_translation(), f_h.get_orientation(),
                         body, osim.Vec3(0), osim.Vec3(0), st)
    m.addJoint(j)  # coordinates exist only after addJoint()
    builder.info["joints"].append(j.getName())
    for k in range(j.numCoordinates()):
        c = j.upd_coordinates(k)
        lim = np.pi if c.getMotionType() == 1 else 0.5
        c.set_range(0, -lim)
        c.set_range(1, lim)
        c.setDefaultClamped(False)
        builder.info["coordinates"].append(c.getName())
    return body


def _add_friction(model, name, sl, radius):
    """Smooth Coulomb friction (+ viscous term) on the slide and twist coordinates."""
    Fc, vt, c = sl["friction_coefficient"] * sl["normal_force"], sl["transition_velocity"], sl["viscous_damping"]
    r = float(radius)
    for coord, peak, v_t, visc in ((f"{name}_slide", Fc, vt, c), (f"{name}_twist", Fc * r, vt / r, c * r * r)):
        f = osim.ExpressionBasedCoordinateForce(coord, f"-{peak!r}*tanh(qdot/{v_t!r}) - {visc!r}*qdot")
        f.setName(f"{coord}_friction")
        model.addForce(f)


@functools.lru_cache(maxsize=64)
def contact_curve(stiffness, radius):
    """(depth, force, energy) of a SmoothSphereHalfSpaceForce with default smoothing.

    Sampled on a one-sphere probe model, so it matches OpenSim's force law exactly
    (Hertz + smoothing); depths from -radius/2 (no contact) to radius. ``energy`` is
    the integral of the force, used by kinematics.py to fit the exo pose.
    """
    m = osim.Model()
    m.setGravity(osim.Vec3(0))
    b = osim.Body("b", 1.0, osim.Vec3(0), osim.Inertia(1.0))
    m.addBody(b)
    m.addJoint(osim.SliderJoint("j", m.getGround(), osim.Vec3(0), osim.Vec3(0, 0, np.pi / 2),
                                b, osim.Vec3(0), osim.Vec3(0, 0, np.pi / 2)))  # slides along Y
    sph = osim.ContactSphere(radius, osim.Vec3(0), b, "s")
    hs = osim.ContactHalfSpace(osim.Vec3(0), osim.Vec3(0, 0, -np.pi / 2), m.getGround(), "h")
    m.addContactGeometry(sph)
    m.addContactGeometry(hs)
    f = osim.SmoothSphereHalfSpaceForce("f", sph, hs)
    f.set_stiffness(stiffness)
    f.set_dissipation(0.0)
    m.addForce(f)
    s = m.initSystem()
    q = m.getCoordinateSet().get(0)
    depth = np.concatenate([np.linspace(-radius / 2, 0, 50, endpoint=False),
                            np.geomspace(radius * 1e-4, radius, 400)])
    force = []
    for d in depth:
        q.setValue(s, radius - d)
        m.realizeDynamics(s)
        force.append(f.getRecordValues(s).get(1))
    force = np.maximum.accumulate(np.array(force))
    energy = np.concatenate([[0.0], np.cumsum(0.5 * (force[1:] + force[:-1]) * np.diff(depth))])
    return depth, force, energy


def contact_depth(stiffness, radius, force):
    """Indentation at which a pad of this stiffness and radius pushes with ``force``."""
    d, F, _ = contact_curve(stiffness, radius)
    if F[-1] < force:
        raise ValueError(f"contact: preload {force} N needs more than the pad radius of indentation; "
                         f"raise the stiffness or the pad radius")
    return float(np.interp(force, F, d))


def _add_contact_pads(builder, name, side, human_body, exo_body, center_g, R_g, spec, radius):
    """Rows of pads around the limb: sphere on the exo, tangent half-space on the human."""
    m, L = builder.model, builder.L
    ct = spec["contact"]
    R_h, p_h = L.body_pose[human_body.getName()]
    R_e, p_e = builder.exo_pose[exo_body.getName()]
    lat = R_g.T @ L.lateral(side)  # lateral (bar side) direction in the cuff frame
    lat_angle = np.arctan2(lat[2], lat[0])
    n_around, rows, rp = int(ct["pads_around"]), int(ct["rows"]), float(ct["pad_radius"])
    half_arc = np.radians(ct["shell_arc"]) / 2
    y_axis = R_g[:, 1]
    pads = []
    for row in range(rows):
        y = spec["width"] * ((row + 0.5) / rows - 0.5)
        for k in range(n_around):
            phi = lat_angle + 2 * np.pi * k / n_around
            gap = abs((phi - lat_angle + np.pi) % (2 * np.pi) - np.pi)
            shell = gap <= half_arc + 1e-9
            E = float(ct["stiffness_shell"] if shell else ct["stiffness_strap"])
            depth = contact_depth(E, rp, float(ct["preload_force"]))
            n_g = R_g @ np.array([np.cos(phi), 0.0, np.sin(phi)])  # outward normal of the skin
            skin_g = center_g + y * y_axis + radius * n_g
            pname = f"{name}_pad{row}_{k}"
            # ContactHalfSpace: solid on the +X side of its frame, so X points into the limb.
            R_hs = np.column_stack([-n_g, y_axis, np.cross(-n_g, y_axis)])
            hs = osim.ContactHalfSpace(vec3(R_h.T @ (skin_g - p_h)), vec3(body_xyz_angles(R_h.T @ R_hs)),
                                       human_body, f"{pname}_skin")
            sph = osim.ContactSphere(rp, vec3(R_e.T @ (skin_g + (rp - depth) * n_g - p_e)), exo_body,
                                     f"{pname}_sphere")
            m.addContactGeometry(hs)
            m.addContactGeometry(sph)
            f = osim.SmoothSphereHalfSpaceForce(pname, sph, hs)
            f.set_stiffness(E)
            for prop in ("dissipation", "static_friction", "dynamic_friction", "viscous_friction",
                         "transition_velocity"):
                getattr(f, f"set_{prop}")(float(ct[prop]))
            m.addForce(f)
            pads.append({"name": pname, "shell": bool(shell), "depth": float(depth)})
    builder.info["contacts"].append({"cuff": name, "pads": pads})


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
