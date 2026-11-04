"""Cuff models: ring (series springs), sliding (friction) and contact (unilateral pads)."""
import opensim as osim
import numpy as np
import pytest

from exo_builder import build_exo
from exo_builder import config as C
from exo_builder.analysis import coupling_loads, coupling_loads_over_motion
from exo_builder.interaction import contact_depth
from exo_builder.kinematics import ExoFitter, fit_exo_to_motion

HIP_ONLY = {"segments": {"shank": {"enabled": False}, "foot": {"enabled": False}}}


def hip_cfg(model, **cuff):
    return dict(HIP_ONLY, cuff_defaults=dict(model=model, **cuff))


def set_deg(model, s, **q):
    cs = model.getCoordinateSet()
    for k, v in q.items():
        cs.get(k).setValue(s, float(np.radians(v)), False)


@pytest.mark.parametrize("model_name", C.CUFF_MODELS)
def test_every_cuff_model_builds_unloaded(model_path, model_name):
    model, info = build_exo(model_path, hip_cfg(model_name))
    s = model.initSystem()
    loads = coupling_loads(model, s)
    assert loads["thigh_cuff_r"]["F_total"] < 0.05  # contact: equal, opposed pad preloads
    assert abs(loads["thigh_cuff_r"]["M_axial"]) < 1e-3
    fs = model.getForceSet()
    if model_name in ("ring", "sliding"):
        assert model.getBodySet().contains("exo_thigh_cuff_r_ring")
    if model_name == "ring":
        assert fs.contains("thigh_cuff_r_strap")
    if model_name == "sliding":
        assert fs.contains("thigh_cuff_r_slide_friction") and fs.contains("thigh_cuff_r_twist_friction")
    if model_name == "contact":
        assert sum(1 for f in fs if f.getName().startswith("thigh_cuff_r_pad")) == 16
    # The cuff mass moves to the ring, it is not added twice.
    assert info["exo_mass"] == pytest.approx(build_exo(model_path, HIP_ONLY)[1]["exo_mass"])


def test_ring_strap_and_tissue_in_series(model_path):
    """With the exo held still, the ring settles where strap and tissue carry the same load,
    so the cuff is softer than the tissue alone: k = k_t k_s / (k_t + k_s)."""
    k_s = 4000.0
    strap = {"stiffness_translational": {"anteroposterior": k_s, "longitudinal": 5e4, "mediolateral": k_s}}
    loads = {}
    for tag, cfg in (("lin", HIP_ONLY), ("ring", hip_cfg("ring", ring={"strap": strap}))):
        model, _ = build_exo(model_path, cfg)
        s = model.initSystem()
        fitter = ExoFitter(model, s)
        keep = [i for i, n in enumerate(fitter.names) if "_ring_" in n]  # fit the rings only
        fitter.names = [fitter.names[i] for i in keep]
        fitter.coords = [fitter.coords[i] for i in keep]
        fitter.bounds = tuple(b[keep] for b in fitter.bounds)
        set_deg(model, s, hip_flexion_r=3)
        fitter.fit(s)
        loads[tag] = coupling_loads(model, s)["thigh_cuff_r"]["F_ap"]
    k_t = C.DEFAULTS["cuff_defaults"]["stiffness_translational"]["anteroposterior"]
    assert loads["ring"] == pytest.approx(loads["lin"] * k_s / (k_t + k_s), rel=0.1)


def _hang(no_muscle_model, normal_force, t_end=0.25):
    """knee_right with sliding cuffs hanging on the locked human; returns the cuff slides."""
    cfg = C.load_config("knee_right")
    cfg["cuff_defaults"]["model"] = "sliding"
    cfg["cuff_defaults"]["sliding"]["normal_force"] = normal_force
    model, info = build_exo(no_muscle_model, cfg)
    for c in model.getCoordinateSet():
        if not c.getJoint().getChildFrame().findBaseFrame().getName().startswith("exo_"):
            c.setDefaultLocked(True)
    s = model.initSystem()
    manager = osim.Manager(model)
    manager.setIntegratorAccuracy(1e-4)
    manager.initialize(s)
    s = manager.integrate(t_end)
    cs = model.getCoordinateSet()
    return [cs.get(n).getValue(s) for n in info["coordinates"] if n.endswith("_slide")]


def test_sliding_cuff_friction_holds_or_slips(no_muscle_model):
    # Exo weight ~25 N on three cuffs: mu*N = 30 N each holds it, 1.5 N each does not.
    held = _hang(no_muscle_model, 60.0)
    slipped = _hang(no_muscle_model, 3.0)
    assert len(held) == 3
    assert max(abs(x) for x in held) < 0.005
    assert min(slipped) < -0.01  # the rings (and the exo) slide down the limb


def test_contact_pads_are_compression_only(model_path):
    model, info = build_exo(model_path, hip_cfg("contact"))
    s = model.initSystem()
    fs = model.getForceSet()
    pads = [p["name"] for p in info["contacts"][0]["pads"]]

    def normal_forces():
        model.realizeDynamics(s)
        return np.array([np.linalg.norm([fs.get(n).getRecordValues(s).get(i) for i in range(3)]) for n in pads])

    rest = normal_forces()
    preload = C.DEFAULTS["cuff_defaults"]["contact"]["preload_force"]
    assert rest == pytest.approx(preload, rel=0.01)
    set_deg(model, s, hip_flexion_r=8)  # the limb moves inside the still cuff
    moved = normal_forces()
    assert moved.min() < 0.05 * preload  # pads on one side lift off (they cannot pull)
    assert moved.max() > 3 * preload
    F = coupling_loads(model, s)["thigh_cuff_r"]
    assert F["F_ap"] < -50  # the cuff resists the thigh moving forward


def test_contact_depth_matches_force_law():
    d = contact_depth(5e5, 0.01, 10.0)
    assert 0 < d < 0.01
    assert contact_depth(2e6, 0.01, 10.0) < d  # stiffer pad, smaller indentation


def test_nonlinear_without_cubic_fits_like_linear(model_path, motion_path):
    """Regression: the fitter used fixed weights for ExpressionBasedBushingForce."""
    peaks = {}
    for tag, model_name in (("lin", "linear"), ("nl", "nonlinear")):
        model, _ = build_exo(model_path, {"cuff_defaults": {"model": model_name}})
        time, table, _ = fit_exo_to_motion(model, motion_path, step=40)
        _, loads = coupling_loads_over_motion(model, table)
        peaks[tag] = np.max(np.abs(loads["shank_cuff_r"]["F_ml"]))
    assert peaks["nl"] == pytest.approx(peaks["lin"], rel=0.02)


@pytest.mark.parametrize("bad", [
    {"model": "contact", "contact": {"pads_around": 2}},
    {"model": "contact", "contact": {"preload_force": 0}},
    {"model": "contact", "contact": {"shell_arc": 400}},
    {"model": "sliding", "sliding": {"transition_velocity": 0}},
    {"model": "ring", "ring": {"mass": -1}},
    {"model": "weld"},
])
def test_invalid_cuff_parameters(bad):
    with pytest.raises(C.ConfigError):
        C.load_config({"cuff_defaults": bad})
    with pytest.raises(C.ConfigError, match="width"):
        C.load_config({"segments": {"thigh": {"cuffs": [{"name": "c", "width": 0}]}}})


@pytest.mark.parametrize("cuff,min_load,tol", [
    # ignoring the cubic term in the fit leaves ~0.1 N.m here
    ({"model": "nonlinear", "stiffness_cubic": {"anteroposterior": 2e7, "longitudinal": 0, "mediolateral": 2e7}}, 5, 0.01),
    # the pads slide along the limb and relieve most of the load; the sampled force law leaves
    # a few hundredths of N.m
    ({"model": "contact"}, 1, 0.1),
])
def test_fit_is_a_static_equilibrium(model_path, cuff, min_load, tol):
    """At the fitted pose the coupling forces must balance on every exo coordinate (zero speeds,
    no gravity): inverse dynamics with zero accelerations needs ~0 there, also with cubic terms
    and unilateral pads, whose true force laws differ from a quadratic energy."""
    cfg = dict(HIP_ONLY, cuff_defaults=cuff, pelvis_unit={"attach": {"model": "weld"}},
               joints={"hip": {"misalignment": [0.0, 0.03, 0.0]}})
    model, _ = build_exo(model_path, cfg)
    model.initSystem()  # setGravity on the freshly built model (stale System) segfaults
    model.setGravity(osim.Vec3(0))
    s = model.initSystem()
    set_deg(model, s, hip_flexion_r=40)
    fitter = ExoFitter(model, s)
    fitter.fit(s)
    assert coupling_loads(model, s)["thigh_cuff_r"]["F_total"] > min_load  # misaligned: loaded
    tau = osim.InverseDynamicsSolver(model).solve(s, osim.Vector(s.getNU(), 0.0))
    cs = model.getCoordinateSet()
    for name in fitter.names:
        cs.get(name).setSpeedValue(s, 1.0)
        i = int(np.argmax([abs(s.getU().get(k)) for k in range(s.getNU())]))
        cs.get(name).setSpeedValue(s, 0.0)
        assert abs(tau.get(i)) < tol, name


def test_yaml_reads_exponent_floats(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text("cuff_defaults:\n  model: contact\n  contact: {stiffness_shell: 4e4, stiffness_strap: 1.5E4}\n")
    c = C.load_config(str(p))["cuff_defaults"]["contact"]
    assert c["stiffness_shell"] == 4e4 and c["stiffness_strap"] == 1.5e4


def test_unknown_key_in_model_block():
    with pytest.raises(C.ConfigError, match="unknown keys"):
        C.load_config({"cuff_defaults": {"model": "contact", "contact": {"preload": 3}}})
