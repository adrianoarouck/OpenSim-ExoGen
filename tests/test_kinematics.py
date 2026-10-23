import opensim as osim
import numpy as np
import pytest

from exo_builder import build_exo
from exo_builder.analysis import coupling_loads_over_motion, summarize
from exo_builder.kinematics import exo_coordinates, fit_exo_to_motion


def test_exo_coordinates_detected(model_path):
    model, info = build_exo(model_path, "full_bilateral")
    model.initSystem()
    assert set(exo_coordinates(model)) == set(info["coordinates"])


def test_hip_exo_follows_gait_exactly(model_path, motion_path, tmp_path):
    model, _ = build_exo(model_path, "hip_bilateral")
    model.initSystem()
    out = tmp_path / "fit.mot"
    time, table, energy = fit_exo_to_motion(model, motion_path, out, step=4)
    assert out.exists()
    assert energy.max() < 1e-8  # 3-DOF hip + free belt can match the human exactly
    t, loads = coupling_loads_over_motion(model, table)
    peaks = summarize(loads)
    # Only finite-difference error remains, even with damping (u = N^-1 qdot for the FreeJoint).
    assert max(v["F_total"]["peak"] for v in peaks.values()) < 2.0


def test_full_exo_fit_is_bounded(model_path, motion_path):
    model, _ = build_exo(model_path, "full_bilateral")
    model.initSystem()
    time, table, energy = fit_exo_to_motion(model, motion_path, step=10)
    assert np.all(np.isfinite(energy))
    assert energy.max() < 20.0  # J; kinematic incompatibility of pin knee vs walker knee
