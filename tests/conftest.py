import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import exo_builder  # noqa: E402,F401  FIRST import (opensim/numpy/Moco DLL setup)

import opensim as osim  # noqa: E402
import pytest  # noqa: E402
MODEL = str(ROOT / "subject_walk_contact.osim")
MOTION = str(ROOT / "Walking-tests" / "MOCAP-IK" / "walking1.mot")
osim.Logger.setLevelString("error")


@pytest.fixture(scope="session")
def model_path():
    return MODEL


@pytest.fixture(scope="session")
def motion_path():
    return MOTION


@pytest.fixture(scope="session")
def no_muscle_model(tmp_path_factory):
    """Human model without muscles (faster forward simulations)."""
    proc = osim.ModelProcessor(MODEL)
    proc.append(osim.ModOpRemoveMuscles())
    m = proc.process()
    path = tmp_path_factory.mktemp("models") / "human_nomuscle.osim"
    m.printToXML(str(path))
    return str(path)
