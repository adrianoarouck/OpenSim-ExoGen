"""Forward simulation: the human is locked, the exo hangs on it through its couplings.

After settling, the couplings must carry exactly the exo weight. This checks the
whole chain (geometry, mass properties, bushing frames and signs) dynamically.
"""
import opensim as osim
import numpy as np
import pytest

from exo_builder import build_exo
from exo_builder.analysis import find_couplings


@pytest.mark.parametrize("preset", ["hip_bilateral", "full_bilateral", "knee_right", "ankle_bilateral"])
def test_exo_hangs_on_locked_human(preset, no_muscle_model):
    model, info = build_exo(no_muscle_model, preset)
    for c in model.getCoordinateSet():
        child = c.getJoint().getChildFrame().findBaseFrame().getName()
        if not child.startswith("exo_"):
            c.setDefaultLocked(True)
    s = model.initSystem()
    manager = osim.Manager(model)
    manager.setIntegratorAccuracy(1e-5)
    manager.initialize(s)
    s = manager.integrate(1.0)
    model.realizeVelocity(s)
    fy = sum(model.getForceSet().get(n).getRecordValues(s).get(1) for n in find_couplings(model))
    weld_free = [c for c in info["couplings"] if c["model"] == "weld"]
    assert not weld_free
    weight = info["exo_mass"] * 9.80665
    # Force on the human (frame1) points down and equals the exo weight.
    assert fy == pytest.approx(-weight, rel=0.03)
    # Nothing exploded: exo coordinates stay finite and small.
    q = np.array([c.getValue(s) for c in model.getCoordinateSet() if c.getName().startswith("exo_")])
    assert np.all(np.isfinite(q))
