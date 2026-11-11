"""Human size: scale the model to a stature / hip width / mass (ANSUR II presets)."""
import opensim as osim
import pytest

from exo_builder import build_exo
from exo_builder import config as C
from exo_builder.human import HumanLandmarks, reference_size


def measured(model_path_or_model, cfg):
    model = osim.Model(model_path_or_model) if isinstance(model_path_or_model, str) else model_path_or_model
    return reference_size(HumanLandmarks(model, cfg), cfg)


def test_no_size_keeps_the_model(model_path):
    _, info = build_exo(model_path, {"segments": {"shank": {"enabled": False}, "foot": {"enabled": False}}})
    hs = info["human_size"]
    assert hs["factors"] == {"height": 1.0, "width": 1.0}
    assert hs["size"] == hs["reference"]
    assert 1.6 < hs["reference"]["height"] < 1.95 and 50 < hs["reference"]["mass"] < 120


@pytest.mark.parametrize("preset", list(C.HUMAN_SIZE_PRESETS))
def test_presets_scale_the_human(model_path, preset):
    want = C.HUMAN_SIZE_PRESETS[preset]
    cfg = C.load_config({"human": {"size": dict(want)}})
    model, info = build_exo(model_path, cfg)
    assert info["human_size"]["size"] == pytest.approx(want)
    human_mass = sum(b.getMass() for b in model.getBodySet() if not b.getName().startswith("exo_"))
    assert human_mass == pytest.approx(want["mass"], rel=1e-6)
    # Measure the scaled human the same way the reference was measured (exo removed).
    scaled = osim.Model(model_path)
    from exo_builder.human import scale_human
    scale_human(scaled, cfg)
    got = measured(scaled, cfg)
    assert got["height"] == pytest.approx(want["height"], rel=0.01)
    assert got["hip_width"] == pytest.approx(want["hip_width"], rel=0.05)  # proximal radius follows the length


def test_only_height_given(model_path):
    cfg = C.load_config({"human": {"size": {"height": 1.9}}})
    _, info = build_exo(model_path, cfg)
    hs = info["human_size"]
    assert hs["factors"]["height"] == pytest.approx(1.9 / hs["reference"]["height"])
    assert hs["factors"]["width"] == 1.0 and hs["size"]["mass"] == hs["reference"]["mass"]


@pytest.mark.parametrize("bad", [{"height": 0}, {"mass": -70}, {"weight": 70}])
def test_invalid_size(bad):
    with pytest.raises(C.ConfigError):
        C.load_config({"human": {"size": bad}})
