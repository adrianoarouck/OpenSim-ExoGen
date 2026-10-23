"""Human-exo interaction loads (cuffs, belt, foot binding).

Each coupling is a BushingForce / ExpressionBasedBushingForce whose frame1 is on
the human. Loads are reported as the force/moment the exo applies ON THE HUMAN,
expressed in the human-side coupling frame at its origin:

    F_ap  (+ anterior)   F_axial (+ proximal / up)   F_ml (+ right)
    M_ap, M_axial, M_ml  (about the coupling centre)
    F_lat (+ lateral for that side), F_radial = sqrt(F_ap^2 + F_ml^2)

Implementation notes (verified on OpenSim 4.5 and 4.6):
* getRecordValues(state) gives 12 numbers: force and torque on the frame1 body,
  then on the frame2 body, in ground; the torque is about the BODY ORIGIN, so it
  is shifted to the coupling centre here. Stage Velocity is enough.
"""
import numpy as np
import opensim as osim

from .geometry import rot_to_np, to_np

LOAD_KEYS = ("F_ap", "F_axial", "F_ml", "M_ap", "M_axial", "M_ml", "F_lat", "F_radial", "F_total")
BUSHING_TYPES = ("BushingForce", "ExpressionBasedBushingForce")


def find_couplings(model):
    """Names of every bushing-type force in the model (cuffs, belt, bindings...)."""
    return [f.getName() for f in model.getForceSet() if f.getConcreteClassName() in BUSHING_TYPES]


def _side_sign(name):
    return -1.0 if name.endswith("_l") else 1.0


def coupling_loads(model, state, names=None):
    """Loads of each coupling at ``state`` (needs at least Stage::Velocity). Returns {name: dict}."""
    model.realizeVelocity(state)
    out = {}
    fs = model.getForceSet()
    for name in names or find_couplings(model):
        f = fs.get(name)
        cls = getattr(osim, f.getConcreteClassName()).safeDownCast(f)  # owned by the model: safe
        rec = f.getRecordValues(state)
        F_g = np.array([rec.get(i) for i in range(3)])
        T_g = np.array([rec.get(i) for i in range(3, 6)])
        frame1 = cls.getFrame1()
        T1 = frame1.getTransformInGround(state)
        R1, p1 = rot_to_np(T1.R()), to_np(T1.p())
        p_body = to_np(frame1.findBaseFrame().getTransformInGround(state).p())
        M_g = T_g - np.cross(p1 - p_body, F_g)  # shift torque from body origin to coupling centre
        F, M = R1.T @ F_g, R1.T @ M_g
        defl = to_np(cls.computeDeflection(state))  # [theta_x, theta_y, theta_z, dx, dy, dz]
        out[name] = {"F_ap": F[0], "F_axial": F[1], "F_ml": F[2], "M_ap": M[0], "M_axial": M[1],
                     "M_ml": M[2], "F_lat": _side_sign(name) * F[2], "F_radial": float(np.hypot(F[0], F[2])),
                     "F_total": float(np.linalg.norm(F)), "deflection": defl}
    return out


# --------------------------------------------------------------- motion I/O
def read_coordinates(path_or_table, model):
    """Read a .mot/.sto of coordinates -> (time, {coord_name: values in model units})."""
    table = path_or_table if isinstance(path_or_table, osim.TimeSeriesTable) else osim.TimeSeriesTable(str(path_or_table))
    in_degrees = table.hasTableMetaDataKey("inDegrees") and \
        table.getTableMetaDataAsString("inDegrees").strip().lower() == "yes"
    time = np.array(table.getIndependentColumn())
    coords = model.getCoordinateSet()
    data = {}
    for label in table.getColumnLabels():
        name = label
        if label.startswith("/"):
            parts = label.split("/")
            if parts[-1] != "value":
                continue
            name = parts[-2]
        if not coords.contains(name):
            continue
        col = table.getDependentColumn(label).to_numpy().copy()
        if in_degrees and coords.get(name).getMotionType() == 1:
            col = np.radians(col)
        data[name] = col
    return time, data


def iterate_states(model, time, data, speeds=True, assemble=True):
    """Yield (t, state) with coordinates from ``data`` and finite-difference speeds.

    Speeds are computed as qdot of the full state Q vector and mapped with
    u = N^-1 qdot: for FreeJoint/BallJoint rotations u is an angular velocity,
    not the derivative of the coordinate, so Coordinate.setSpeedValue(dq/dt)
    would be wrong there.
    """
    state = model.initSystem()
    coords = model.getCoordinateSet()
    Q = []
    for i in range(len(time)):
        for name, v in data.items():
            coords.get(name).setValue(state, float(v[i]), False)
        if assemble and model.getConstraintSet().getSize() > 0:
            model.assemble(state)
        Q.append(to_np(state.getQ()))
    Q = np.array(Q)
    Qdot = np.gradient(Q, time, axis=0) if speeds and len(time) > 1 else None
    matter = model.getMatterSubsystem()
    for i, t in enumerate(time):
        state.setTime(float(t))
        state.setQ(osim.Vector(Q[i].tolist()))
        model.realizePosition(state)
        if Qdot is not None:
            u = osim.Vector(state.getNU(), 0.0)
            matter.multiplyByNInv(state, False, osim.Vector(Qdot[i].tolist()), u)
            state.setU(u)
        yield t, state


def coupling_loads_over_motion(model, motion, names=None, speeds=True):
    """Loads of every coupling along a motion (.mot/.sto path or TimeSeriesTable).

    The motion must contain the exo coordinates too (e.g. from
    kinematics.fit_exo_to_motion or a Moco solution), otherwise the exo stays at
    its default pose and the loads are meaningless.
    """
    time, data = read_coordinates(motion, model)
    names = names or find_couplings(model)
    loads = {n: {k: [] for k in LOAD_KEYS} for n in names}
    for _, state in iterate_states(model, time, data, speeds=speeds):
        for n, d in coupling_loads(model, state, names).items():
            for k in LOAD_KEYS:
                loads[n][k].append(d[k])
    return time, {n: {k: np.array(v) for k, v in d.items()} for n, d in loads.items()}


def loads_to_table(time, loads):
    """Loads dict -> osim.TimeSeriesTable with columns '<coupling>/<component>'."""
    labels = [f"{n}/{k}" for n in loads for k in LOAD_KEYS]
    mat = np.column_stack([loads[n][k] for n in loads for k in LOAD_KEYS])
    table = osim.TimeSeriesTable()
    table.setColumnLabels(labels)
    for i, t in enumerate(time):
        table.appendRow(float(t), osim.RowVector(mat[i].tolist()))
    return table


def write_loads(time, loads, path):
    osim.STOFileAdapter.write(loads_to_table(time, loads), str(path))


def summarize(loads):
    """Peak / RMS per coupling and component."""
    return {n: {k: {"peak": float(np.max(np.abs(v))), "rms": float(np.sqrt(np.mean(v ** 2)))}
                for k, v in d.items()} for n, d in loads.items()}


def plot_loads(time, loads, path=None, components=("F_ap", "F_ml", "F_axial", "M_axial")):
    import matplotlib
    if path:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(components), 1, sharex=True, figsize=(9, 2.2 * len(components)))
    for ax, comp in zip(np.atleast_1d(axes), components):
        for n, d in loads.items():
            ax.plot(time, d[comp], label=n)
        ax.set_ylabel(comp + (" [N.m]" if comp.startswith("M") else " [N]"))
        ax.grid(alpha=0.3)
    np.atleast_1d(axes)[0].legend(fontsize=7, ncol=3)
    np.atleast_1d(axes)[-1].set_xlabel("time [s]")
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=130)
        plt.close(fig)
    return fig
