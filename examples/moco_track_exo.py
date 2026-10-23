"""Torque-driven MocoTrack of a human + exo model (smoke test / starting point).

Pipeline:
  1. build the exo around the human model (exo_builder preset or YAML),
  2. fit the exo coordinates to the human IK motion (minimum coupling energy),
  3. track the complete coordinate set with MocoTrack (muscles removed,
     reserves on HUMAN coordinates only; the exo is moved by its own actuators
     and by the cuffs),
  4. compute cuff/belt/binding loads along the Moco solution.

    python examples/moco_track_exo.py --preset hip_bilateral --t0 0.5 --t1 0.8
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import exo_builder  # noqa: E402  FIRST: sets up opensim/numpy/Moco DLLs (see exo_builder/__init__.py)

import argparse  # noqa: E402

import opensim as osim  # noqa: E402

from exo_builder import build_exo  # noqa: E402
from exo_builder.analysis import coupling_loads_over_motion, summarize, write_loads  # noqa: E402
from exo_builder.kinematics import exo_coordinates, fit_exo_to_motion  # noqa: E402


def processed_model(model_path):
    proc = osim.ModelProcessor(str(model_path))
    proc.append(osim.ModOpRemoveMuscles())
    proc.append(osim.ModOpAddReserves(250.0, 1.0))
    model = proc.process()
    model.initSystem()
    # Exo DOFs must not get reserves: passive ones are driven only by the
    # couplings, actuated ones by their own tau_exo_* actuators.
    exo = set(exo_coordinates(model))
    fs = model.updForceSet()
    for name in [f.getName() for f in fs]:
        act = osim.CoordinateActuator.safeDownCast(fs.get(name))
        if act is not None and act.get_coordinate() in exo and not name.startswith("tau_"):
            fs.remove(fs.getIndex(name))  # look the index up each time: it shifts after removals
    model.finalizeConnections()
    return model


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--preset", default="hip_bilateral")
    p.add_argument("--model", default=str(ROOT / "models" / "subject_walk_contact.osim"))
    p.add_argument("--motion", default=str(ROOT / "data" / "walking" / "MOCAP-IK" / "walking1.mot"))
    p.add_argument("--t0", type=float, default=0.5)
    p.add_argument("--t1", type=float, default=0.8)
    p.add_argument("--mesh", type=float, default=0.05)
    p.add_argument("--max-iter", type=int, default=500)
    p.add_argument("--out", default=str(ROOT / "results" / "moco_exo"))
    args = p.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    model, info = build_exo(args.model, args.preset)
    exo_path = out / f"{args.preset}.osim"
    model.printToXML(str(exo_path))
    model = osim.Model(str(exo_path))
    model.initSystem()
    fit_path = out / f"{args.preset}_fit.mot"
    fit_exo_to_motion(model, args.motion, fit_path)

    track = osim.MocoTrack()
    track.setName(f"track_{args.preset}")
    track.setModel(osim.ModelProcessor(processed_model(exo_path)))
    ref = osim.TableProcessor(str(fit_path))
    ref.append(osim.TabOpLowPassFilter(6))
    ref.append(osim.TabOpUseAbsoluteStateNames())
    track.setStatesReference(ref)
    track.set_states_global_tracking_weight(10)
    track.set_allow_unused_references(True)
    track.set_track_reference_position_derivatives(True)
    track.set_apply_tracked_states_to_guess(True)
    track.set_initial_time(args.t0)
    track.set_final_time(args.t1)
    track.set_mesh_interval(args.mesh)

    study = track.initialize()
    solver = osim.MocoCasADiSolver.safeDownCast(study.updSolver())
    solver.set_optim_max_iterations(args.max_iter)
    solution = study.solve()
    sol_path = out / f"{args.preset}_solution.sto"
    solution.unseal().write(str(sol_path))
    print(f"Moco status: {solution.getStatus()}  objective: {solution.getObjective():.4g}")

    states = solution.exportToStatesTable()
    time, loads = coupling_loads_over_motion(model, states)
    write_loads(time, loads, out / f"{args.preset}_coupling_loads.sto")
    for name, d in summarize(loads).items():
        print(f"  {name:16s} peak |F_ap| {d['F_ap']['peak']:6.1f} N  |F_ml| {d['F_ml']['peak']:6.1f} N")


if __name__ == "__main__":
    main()
