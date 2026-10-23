"""Exo kinematics from human kinematics (replaces the virtual-marker IK hack).

With the human coordinates fixed, the exo coordinates are found by minimising
the elastic energy stored in the couplings,

    E = 1/2 * sum_c ( k_rot,c . theta_c^2 + k_trans,c . delta_c^2 ),

i.e. the static pose a massless, unactuated exo would settle in. Weld
constraints (foot ``weld`` mode) enter with a very high weight. The result is a
complete .mot (human + exo coordinates) for ID/SO, Moco initial guesses or
analysis.coupling_loads_over_motion.
"""
import opensim as osim  # noqa: F401  (import before numpy, see exo_builder/__init__.py)
import numpy as np
from scipy.optimize import least_squares

from .analysis import BUSHING_TYPES, read_coordinates
from .geometry import rot_to_np, to_np

WELD_WEIGHT = (1e5, 1e7)  # rotational, translational "stiffness" for weld constraints


def exo_coordinates(model):
    """Coordinates whose joint moves an exo body (child body name starts with 'exo_')."""
    out = []
    for c in model.getCoordinateSet():
        child = c.getJoint().getChildFrame().findBaseFrame().getName()
        if child.startswith("exo_"):
            out.append(c.getName())
    return out


class _Couplings:
    """Residual vector sqrt(k) * deflection for every coupling of the model."""

    def __init__(self, model):
        self.items = []
        for f in model.getForceSet():
            cls = f.getConcreteClassName()
            if cls not in BUSHING_TYPES:
                continue
            obj = getattr(osim, cls).safeDownCast(f)  # owned by the model: safe
            if cls == "BushingForce":
                w = np.concatenate([to_np(obj.get_rotational_stiffness()),
                                    to_np(obj.get_translational_stiffness())])
            else:  # expression based: use the small-deflection slope of each expression
                w = np.array([30.0, 30.0, 30.0, 3000.0, 3000.0, 3000.0])
            self.items.append(("bushing", obj, np.sqrt(np.maximum(w, 1e-9))))
        for c in model.getConstraintSet():
            weld = osim.WeldConstraint.safeDownCast(c)
            if weld is None:
                continue
            f1 = weld.getSocket("frame1").getConnecteePath()
            f2 = weld.getSocket("frame2").getConnecteePath()
            if "exo" not in f1 + f2:
                continue
            frames = (model.getComponent(f1), model.getComponent(f2))
            frames = tuple(osim.PhysicalFrame.safeDownCast(f) for f in frames)
            w = np.sqrt(np.array([WELD_WEIGHT[0]] * 3 + [WELD_WEIGHT[1]] * 3))
            self.items.append(("weld", frames, w))

    def residual(self, state):
        out = []
        for kind, obj, w in self.items:
            if kind == "bushing":
                d = to_np(obj.computeDeflection(state))
            else:
                f1, f2 = obj
                T = f1.findTransformBetween(state, f2)
                R = rot_to_np(T.R())
                ang = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
                d = np.concatenate([ang, to_np(T.p())])
            out.append(w * d)
        return np.concatenate(out) if out else np.zeros(0)


class ExoFitter:
    """Reusable solver: ``fit(state)`` moves the exo coordinates of ``state`` in place."""

    def __init__(self, model, state=None):
        self.model = model
        self.state = state if state is not None else model.initSystem()
        cs = model.getCoordinateSet()
        self.names = exo_coordinates(model)
        self.coords = [cs.get(n) for n in self.names]
        self.couplings = _Couplings(model)
        lo, hi = [], []
        for c in self.coords:
            clamp = c.getDefaultClamped()
            lo.append(c.getRangeMin() if clamp else -np.inf)
            hi.append(c.getRangeMax() if clamp else np.inf)
        self.bounds = (np.array(lo), np.array(hi))

    def _set(self, state, x):
        for c, v in zip(self.coords, x):
            c.setValue(state, float(v), False)
        self.model.realizePosition(state)

    def _fun(self, x, state):
        self._set(state, x)
        return self.couplings.residual(state)

    def fit(self, state=None, x0=None):
        state = state if state is not None else self.state
        x0 = np.array([c.getValue(state) for c in self.coords]) if x0 is None else np.asarray(x0)
        x0 = np.clip(x0, self.bounds[0] + 1e-9, self.bounds[1] - 1e-9)
        if not self.coords or not self.couplings.items:
            return x0, 0.0
        sol = least_squares(self._fun, x0, args=(state,), bounds=self.bounds, method="trf",
                            x_scale=1.0, xtol=1e-10, ftol=1e-10, max_nfev=200)
        self._set(state, sol.x)
        return sol.x, float(0.5 * np.sum(sol.fun ** 2))


def fit_exo_to_motion(model, motion, out_path=None, step=1, progress=False):
    """Exo coordinates for every frame of a human motion (.mot/.sto or TimeSeriesTable).

    Returns (time, table) where ``table`` is an osim.TimeSeriesTable with all model
    coordinates (rotations in degrees, IK style). Writes it to ``out_path`` if given.
    """
    time, data = read_coordinates(motion, model)
    time = time[::step]
    data = {k: v[::step] for k, v in data.items()}
    fitter = ExoFitter(model)
    state = fitter.state
    cs = model.getCoordinateSet()
    names = [c.getName() for c in cs]
    rows, energy = [], []
    x = None
    for i, t in enumerate(time):
        state.setTime(float(t))
        for n, v in data.items():
            cs.get(n).setValue(state, float(v[i]), False)
        x, e = fitter.fit(state, x)
        energy.append(e)
        rows.append([cs.get(n).getValue(state) for n in names])
        if progress and i % 50 == 0:
            print(f"  frame {i + 1}/{len(time)}  t={t:.3f}  coupling energy={e:.3g} J", flush=True)
    rows = np.array(rows)
    rot = np.array([cs.get(n).getMotionType() == 1 for n in names])
    rows[:, rot] = np.degrees(rows[:, rot])
    table = osim.TimeSeriesTable()
    table.setColumnLabels(names)
    for t, r in zip(time, rows):
        table.appendRow(float(t), osim.RowVector(r.tolist()))
    table.addTableMetaDataString("inDegrees", "yes")
    if out_path:
        osim.STOFileAdapter.write(table, str(out_path))
    return time, table, np.array(energy)
