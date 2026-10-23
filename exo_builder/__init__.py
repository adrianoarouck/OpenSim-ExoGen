"""exo_builder: configurable exoskeleton generator for OpenSim human models.

    import exo_builder                  # FIRST import of the script (sets up opensim, see below)
    import opensim as osim
    import numpy as np
    model, info = exo_builder.build_exo("models/subject_walk_contact.osim", "hip_bilateral")
    model.printToXML("subject_hip_exo.osim")

Why it must be the first import (Windows, OpenSim 4.6 from conda; all verified):
* opensim ships libblas.dll / liblapack.dll. If numpy is imported first, its
  MKL DLLs with the same names win, and Model.assemble() /
  Coordinate.setValue() crash with an access violation -> opensim before numpy.
* Moco/CasADi loads its Ipopt plugin by name through PATH. With another OpenSim
  install on PATH (e.g. C:/OpenSim 4.5/bin) it takes that version's plugin:
  "Plugin 'ipopt' is not found" (WIN32 error 127) -> CASADIPATH.
* conda's MKL threads through conda's libiomp5md.dll, an llvm-openmp shim that
  forwards to libomp.dll. OpenSim's Ipopt/MUMPS need Intel's libiomp5md.dll and
  fail with WIN32 error 182 once the shim is loaded -> MKL_THREADING_LAYER=
  SEQUENTIAL, which only works if set BEFORE opensim is imported.
All settings affect this process only and respect values already set.
"""
import os
import sys
import warnings

if sys.platform.startswith("win"):
    if "opensim" in sys.modules and "MKL_THREADING_LAYER" not in os.environ:
        warnings.warn("opensim was imported before exo_builder: Moco solves in this process may fail "
                      "with \"Plugin 'ipopt' is not found\" (WIN32 error 182). Make 'import exo_builder' "
                      "the first import of the script.", RuntimeWarning, stacklevel=2)
    os.environ.setdefault("MKL_THREADING_LAYER", "SEQUENTIAL")
if "numpy" in sys.modules and "opensim" not in sys.modules:
    warnings.warn("numpy was imported before opensim: with OpenSim 4.6 on Windows this can crash "
                  "assemble()/setValue() (BLAS/LAPACK DLL clash). Make 'import exo_builder' (or "
                  "'import opensim') the first import of the script.", RuntimeWarning, stacklevel=2)
import opensim  # noqa: E402,F401  (must come before numpy)

if sys.platform.startswith("win"):
    os.environ.setdefault("CASADIPATH", os.path.dirname(opensim.__file__))

from .builder import ExoBuilder, build_exo  # noqa: E402
from .config import ConfigError, list_presets, load_config, save_config  # noqa: E402

__all__ = ["ExoBuilder", "build_exo", "load_config", "save_config", "list_presets", "ConfigError"]
