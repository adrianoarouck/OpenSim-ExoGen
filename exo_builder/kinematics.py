"""Exo kinematics from human kinematics (replaces the virtual-marker IK hack).

With the human coordinates fixed, the exo coordinates are found by minimising
the elastic energy stored in the couplings,

    E = 1/2 * sum_c ( k_rot,c . theta_c^2 + k_trans,c . delta_c^2 ),

i.e. the static pose a massless, unactuated exo would settle in. Weld
constraints (foot ``weld`` mode) enter with a very high weight. Contact cuff
pads add their Hertz energy plus a tangential "no slip" term, and the slide/twist
coordinates of sliding cuffs are held at their current values: friction is not
an energy, so the fit assumes the cuffs do not slip. The result is a
complete .mot (human + exo coordinates) for ID/SO, Moco initial guesses or
analysis.coupling_loads_over_motion.
"""
import opensim as osim  # noqa: F401  (import before numpy, see exo_builder/__init__.py)
import numpy as np
from scipy.optimize import least_squares

import re

from .analysis import BUSHING_TYPES, read_coordinates
from .geometry import rot_from_xyz, rot_to_np, to_np
from .interaction import PAD_RE, SLIDING_SUFFIXES, contact_curve

WELD_WEIGHT = (1e5, 1e7)  # rotational, translational "stiffness" for weld constraints
FALLBACK_WEIGHT = (30.0, 3000.0)  # custom nonlinear expressions whose slope cannot be read
_NUM = r"([-+]?[0-9.]+(?:[eE][-+]?[0-9]+)?)"
# The builder writes 'k1*var' or 'k1*var + k3*var^3' (interaction.make_coupling).
_POLY_RE = re.compile(rf"^\s*{_NUM}\s*\*\s*(delta|theta)_[xyz]\s*(?:\+\s*{_NUM}\s*\*\s*\2_[xyz]\s*\^\s*3)?\s*$")
TANGENTIAL_FRACTION = 1e-3  # pad "no slip" stiffness / normal slope (only fixes the sliding null space)


def _expression_stiffness(f):
    """(k1, k3) of an ExpressionBasedBushingForce, read from 'k1*var [+ k3*var^3]' (the form
    the builder writes); other custom expressions fall back to FALLBACK_WEIGHT, k3 = 0."""
    k1, k3 = [], []
    for i, key in enumerate(("Mx", "My", "Mz", "Fx", "Fy", "Fz")):
        m = _POLY_RE.match(getattr(f, f"get{key}Expression")())
        k1.append(float(m.group(1)) if m else FALLBACK_WEIGHT[i // 3])
        k3.append(float(m.group(3)) if m and m.group(3) else 0.0)
    return np.array(k1), np.array(k3)


def _pad_energy(curve, depth):
    """Elastic energy of a pad at ``depth`` (Hertz-like extrapolation past the sampled range)."""
    d, F, E = curve
    if depth <= d[-1]:
        return float(np.interp(depth, d, E))
    return float(E[-1] + F[-1] * d[-1] / 2.5 * ((depth / d[-1]) ** 2.5 - 1.0))


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

    def __init__(self, model, state):
        self.items = []
        for f in model.getForceSet():
            cls = f.getConcreteClassName()
            if cls not in BUSHING_TYPES:
                continue
            obj = getattr(osim, cls).safeDownCast(f)  # owned by the model: safe
            if cls == "BushingForce":
                k1 = np.concatenate([to_np(obj.get_rotational_stiffness()),
                                     to_np(obj.get_translational_stiffness())])
                k3 = np.zeros(6)
            else:  # expression based: linear and cubic coefficients of each expression
                k1, k3 = _expression_stiffness(obj)
            self.items.append(("bushing", obj, (np.maximum(k1, 1e-9), np.maximum(k3, 0.0))))
        ref = osim.State(state)  # default pose: where the pads sit at their preload
        for c in model.getCoordinateSet():
            c.setValue(ref, c.getDefaultValue(), False)
        model.realizePosition(ref)
        for f in model.getForceSet():
            if f.getConcreteClassName() != "SmoothSphereHalfSpaceForce" or not PAD_RE.match(f.getName()):
                continue
            sshsf = osim.SmoothSphereHalfSpaceForce.safeDownCast(f)
            sph = osim.ContactSphere.safeDownCast(model.getComponent(f.getSocket("sphere").getConnecteePath()))
            hs = osim.ContactHalfSpace.safeDownCast(model.getComponent(f.getSocket("half_space").getConnecteePath()))
            curve = contact_curve(sshsf.get_stiffness(), sph.getRadius())
            pad = (sph, hs, sph.getRadius(), curve)
            p = self._pad_position(ref, pad)
            depth0 = sph.getRadius() + p[0]
            k_n = max(float(np.interp(depth0, curve[0], np.gradient(curve[1], curve[0]))), 1.0)
            # k_n: normal slope at the preload (linearised first pass); the tangential term is
            # weak: smooth friction holds no static load, it only keeps the pads from drifting.
            self.items.append(("pad", pad, (p[1:], np.sqrt(k_n), np.sqrt(TANGENTIAL_FRACTION * k_n), depth0)))
        self.has_pads = any(kind == "pad" for kind, _, _ in self.items)
        self.linear_pads = False  # True: pads as bilateral linear springs (robust first pass)
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

    @staticmethod
    def _pad_position(state, pad):
        """Sphere centre in the half-space frame (+X into the limb)."""
        sph, hs = pad[0], pad[1]
        c_g = sph.getFrame().findStationLocationInGround(state, sph.get_location())
        T = hs.getFrame().getTransformInGround(state)
        R, o = rot_to_np(T.R()), to_np(T.p())
        R_hs = R @ rot_from_xyz(to_np(hs.get_orientation()))  # body-fixed XYZ, like the builder
        return R_hs.T @ (to_np(c_g) - (o + R @ to_np(hs.get_location())))

    def residual(self, state):
        out = []
        for kind, obj, w in self.items:
            if kind == "bushing":
                d = to_np(obj.computeDeflection(state))
                k1, k3 = w
                # residual^2 / 2 = energy of k1*d + k3*d^3, i.e. k1 d^2/2 + k3 d^4/4
                out.append(d * np.sqrt(k1 + 0.5 * k3 * d * d))
                continue
            elif kind == "pad":
                p = self._pad_position(state, obj)
                yz0, sn, st, depth0 = w
                if self.linear_pads:
                    normal = sn * (obj[2] + p[0] - depth0)
                else:  # residual sqrt(2 E): least squares minimises the contact energy
                    normal = np.sqrt(2.0 * _pad_energy(obj[3], obj[2] + p[0]))
                out.append(np.concatenate([[normal], st * (p[1:] - yz0)]))
                continue
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
        self.names = [n for n in exo_coordinates(model) if not n.endswith(SLIDING_SUFFIXES)]
        self.coords = [cs.get(n) for n in self.names]
        self.couplings = _Couplings(model, self.state)
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
        if self.couplings.has_pads:
            # Contact pads: the unilateral Hertz energy is flat where pads lift off, so first
            # solve with the pads linearised about their preload, then refine with Hertz.
            self.couplings.linear_pads = True
            x0 = least_squares(self._fun, x0, args=(state,), bounds=self.bounds, method="trf",
                               x_scale=1.0, xtol=1e-10, ftol=1e-10, max_nfev=200).x
            self.couplings.linear_pads = False
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
