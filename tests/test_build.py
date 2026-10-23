import opensim as osim
import pytest

from exo_builder import ConfigError, build_exo, list_presets, load_config
from exo_builder.analysis import coupling_loads

EXPECTED = {
    "full_bilateral": {"bodies": 7, "actuators": 6, "couplings": 7},
    "hip_bilateral": {"bodies": 3, "actuators": 2, "couplings": 3},
    "knee_right": {"bodies": 2, "actuators": 1, "couplings": 3},
    "ankle_bilateral": {"bodies": 4, "actuators": 2, "couplings": 4},
    "hip_knee_left": {"bodies": 3, "actuators": 2, "couplings": 3},
    "full_plate_contact": {"bodies": 7, "actuators": 6, "couplings": 7},
}


def test_all_presets_are_covered():
    assert set(list_presets()) == set(EXPECTED)


@pytest.mark.parametrize("preset", sorted(EXPECTED))
def test_preset_builds_with_zero_preload(preset, model_path, tmp_path):
    model, info = build_exo(model_path, preset)
    s = model.initSystem()
    exp = EXPECTED[preset]
    assert len(info["bodies"]) == exp["bodies"]
    assert len(info["actuators"]) == exp["actuators"]
    assert len(info["couplings"]) == exp["couplings"]
    names = [c.getName() for c in model.getCoordinateSet()]
    assert len(names) == len(set(names))
    loads = coupling_loads(model, s)
    assert max(d["F_total"] for d in loads.values()) < 1e-6
    # XML round trip
    out = tmp_path / f"{preset}.osim"
    model.printToXML(str(out))
    m2 = osim.Model(str(out))
    m2.initSystem()
    assert m2.getNumCoordinates() == model.getNumCoordinates()


def test_unilateral_has_only_one_side(model_path):
    model, info = build_exo(model_path, "hip_knee_left")
    assert all(b.endswith("_l") or b == "exo_pelvis" for b in info["bodies"])


def test_floating_root_when_parent_disabled(model_path):
    model, info = build_exo(model_path, "knee_right")
    assert "exo_thigh_r_free" in info["joints"]
    assert any("floating root" in w for w in info["warnings"])


def test_mass_matches_config(model_path):
    model, info = build_exo(model_path, "full_bilateral")
    cfg = load_config("full_bilateral")
    seg = cfg["segments"]
    expected = cfg["pelvis_unit"]["mass"] + cfg["pelvis_unit"]["backpack"]["mass"] + \
        2 * (seg["thigh"]["mass"] + seg["shank"]["mass"] + seg["foot"]["mass"])
    assert info["exo_mass"] == pytest.approx(expected)


def test_exo_axes_follow_human_coordinates(model_path):
    """Moving a human coordinate and its exo twin together keeps the cuffs unloaded."""
    model, _ = build_exo(model_path, "full_bilateral")
    s = model.initSystem()
    cs = model.getCoordinateSet()
    for human, exo in [("hip_flexion_r", "exo_hip_flexion_r"), ("hip_adduction_l", "exo_hip_adduction_l"),
                       ("hip_rotation_r", "exo_hip_rotation_r")]:
        cs.get(human).setValue(s, 0.4, False)
        cs.get(exo).setValue(s, 0.4, False)
    loads = coupling_loads(model, s)
    assert max(d["F_total"] for d in loads.values()) < 1e-6


def test_config_validation_errors():
    with pytest.raises(ConfigError, match="sides"):
        load_config({"sides": ["x"]})
    with pytest.raises(ConfigError, match="at must be"):
        load_config({"segments": {"thigh": {"cuffs": [{"at": 1.5}]}}})
    with pytest.raises(ConfigError, match="unknown variables"):
        load_config({"cuff_defaults": {"model": "nonlinear", "expressions": {"Fx": "1000*deflection"}}})
    with pytest.raises(ConfigError, match="mass must be > 0"):
        load_config({"segments": {"shank": {"mass": 0}}})


def test_side_override(model_path):
    model, info = build_exo(model_path, {"segments": {"shank": {"enabled": False}, "foot": {"enabled": False}},
                                          "overrides": {"l": {"segments": {"thigh": {"mass": 3.0}}}}})
    bs = model.getBodySet()
    assert bs.get("exo_thigh_l").getMass() == pytest.approx(3.0)
    assert bs.get("exo_thigh_r").getMass() == pytest.approx(1.5)
