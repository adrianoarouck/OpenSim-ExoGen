import opensim as osim
import numpy as np
import pytest

from exo_builder import build_exo
from exo_builder.analysis import coupling_loads

HIP_ONLY = {"segments": {"shank": {"enabled": False}, "foot": {"enabled": False}}}


def set_deg(model, s, **q):
    cs = model.getCoordinateSet()
    for k, v in q.items():
        cs.get(k).setValue(s, float(np.radians(v)), False)


def test_knee_mismatch_loads_shank_cuff_anteriorly(model_path):
    model, _ = build_exo(model_path, "full_bilateral")
    s = model.initSystem()
    set_deg(model, s, knee_angle_r=30)  # human flexes, exo knee stays straight
    L = coupling_loads(model, s)
    # The straight exo shank stays in front of the flexed shank: it pushes the human anteriorly.
    assert L["shank_cuff_r"]["F_ap"] > 300
    assert L["thigh_cuff_r"]["F_total"] < 1e-6


def test_misalignment_zero_at_default_loaded_in_flexion(model_path):
    cfg = dict(HIP_ONLY, joints={"hip": {"misalignment": [0.0, 0.03, 0.0]}})
    model, _ = build_exo(model_path, cfg)
    s = model.initSystem()
    assert coupling_loads(model, s)["thigh_cuff_r"]["F_total"] < 1e-6
    set_deg(model, s, hip_flexion_r=40, exo_hip_flexion_r=40)
    assert coupling_loads(model, s)["thigh_cuff_r"]["F_total"] > 10


def test_nonlinear_cuff_matches_linear_for_small_deflection(model_path):
    nl = dict(HIP_ONLY, cuff_defaults={"model": "nonlinear", "cubic": {"ap": 1e6, "axial": 0, "ml": 0}})
    out = {}
    for tag, cfg in (("lin", HIP_ONLY), ("nl", nl)):
        model, _ = build_exo(model_path, cfg)
        s = model.initSystem()
        assert model.getForceSet().get("thigh_cuff_r").getConcreteClassName() == \
            ("BushingForce" if tag == "lin" else "ExpressionBasedBushingForce")
        set_deg(model, s, hip_flexion_r=2)
        small = coupling_loads(model, s)["thigh_cuff_r"]["F_ap"]
        set_deg(model, s, hip_flexion_r=20)
        big = coupling_loads(model, s)["thigh_cuff_r"]["F_ap"]
        out[tag] = (small, big)
    assert out["nl"][0] == pytest.approx(out["lin"][0], rel=0.05)
    assert abs(out["nl"][1]) > abs(out["lin"][1]) * 1.2  # cubic term stiffens large deflections


@pytest.mark.parametrize("mode", ["weld", "bushing", "plate_contact"])
def test_foot_modes(model_path, mode):
    model, info = build_exo(model_path, {"segments": {"foot": {"mode": mode}}})
    model.initSystem()
    if mode == "weld":
        assert model.getConstraintSet().contains("binding_r")
    else:
        assert model.getForceSet().contains("binding_r")
    if mode == "plate_contact":
        f = osim.SmoothSphereHalfSpaceForce.safeDownCast(model.getForceSet().get("heel_r_force"))
        assert f.getSocket("half_space").getConnecteePath().endswith("exo_foot_r_plate")
        g = osim.SmoothSphereHalfSpaceForce.safeDownCast(model.getForceSet().get("exo_heel_r_force"))
        assert g.getSocket("half_space").getConnecteePath().endswith("floor")
        assert g.get_stiffness() == f.get_stiffness()


def test_plate_contact_floor_pushes_plate_up(model_path):
    model, info = build_exo(model_path, "full_plate_contact")
    s = model.initSystem()
    cgs = model.getContactGeometrySet()
    lowest = min(cgs.get(n).getFrame().findStationLocationInGround(s, cgs.get(n).get_location()).get(1)
                 - osim.ContactSphere.safeDownCast(cgs.get(n)).getRadius()
                 for n in [f"exo_{k}_r_geom" for k in ("heel", "mh1", "mh3", "mh5", "hallux")])
    cs = model.getCoordinateSet()
    lift = -lowest - 0.002  # plate spheres 2 mm into the floor
    for name in ("pelvis_ty", "exo_pelvis_ty"):
        cs.get(name).setValue(s, cs.get(name).getValue(s) + lift, False)
    model.realizeVelocity(s)
    assert model.getForceSet().get("exo_heel_r_force").getRecordValues(s).get(1) > 0
