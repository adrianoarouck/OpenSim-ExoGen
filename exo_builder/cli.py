"""Command line interface.

    python -m exo_builder presets
    python -m exo_builder build hip_bilateral --model models/subject_walk_contact.osim -o subject_hip_exo.osim
    python -m exo_builder fit subject_hip_exo.osim data/walking/MOCAP-IK/walking1.mot -o walking1_exo.mot
    python -m exo_builder forces subject_hip_exo.osim walking1_exo.mot -o walking1_cuffs.sto --plot cuffs.png
    python -m exo_builder.viewer --model models/subject_walk_contact.osim --config hip_bilateral
"""
import argparse
import json
from pathlib import Path

import opensim as osim  # noqa: F401  (before numpy)

from . import analysis, config, kinematics
from .builder import build_exo


def _build(args):
    model, info = build_exo(args.model, args.config)
    if args.name:
        model.setName(args.name)
    out = Path(args.out)
    model.printToXML(str(out))
    cfg = config.load_config(args.config)
    config.save_config(cfg, out.with_suffix(".exo.yaml"))
    print(f"wrote {out} (+ {out.with_suffix('.exo.yaml').name}, resolved config)")
    print(f"  exo mass {info['exo_mass']:.2f} kg | bodies {len(info['bodies'])} | "
          f"coordinates {len(info['coordinates'])} | actuators {info['actuators']}")
    print(f"  couplings: {[c['name'] for c in info['couplings']]}")
    for w in info["warnings"]:
        print("  WARNING:", w)
    if args.info:
        Path(args.info).write_text(json.dumps(info, indent=2))


def _fit(args):
    model = osim.Model(args.model)
    model.initSystem()
    _, _, energy = kinematics.fit_exo_to_motion(model, args.motion, args.out, step=args.step, progress=True)
    print(f"wrote {args.out}; coupling energy max {energy.max():.3g} J, mean {energy.mean():.3g} J")


def _forces(args):
    model = osim.Model(args.model)
    model.initSystem()
    time, loads = analysis.coupling_loads_over_motion(model, args.motion, speeds=not args.no_speeds)
    analysis.write_loads(time, loads, args.out)
    print(f"wrote {args.out}")
    for name, d in analysis.summarize(loads).items():
        print(f"  {name:18s} peak |F_ap| {d['F_ap']['peak']:7.1f} N  |F_ml| {d['F_ml']['peak']:7.1f} N  "
              f"|F_axial| {d['F_axial']['peak']:7.1f} N  |M_axial| {d['M_axial']['peak']:6.2f} N.m")
    if args.plot:
        analysis.plot_loads(time, loads, args.plot)
        print(f"wrote {args.plot}")


def main(argv=None):
    p = argparse.ArgumentParser(prog="exo_builder", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("presets", help="list bundled presets")

    b = sub.add_parser("build", help="add an exo to a human model")
    b.add_argument("config", help="preset name or YAML path")
    b.add_argument("--model", required=True, help="human .osim")
    b.add_argument("-o", "--out", required=True, help="output .osim")
    b.add_argument("--name", help="model name inside the .osim")
    b.add_argument("--info", help="write build info JSON here")

    f = sub.add_parser("fit", help="exo coordinates from a human motion (minimum coupling energy)")
    f.add_argument("model", help=".osim with exo")
    f.add_argument("motion", help="human .mot/.sto (IK result)")
    f.add_argument("-o", "--out", required=True, help="output .mot with human + exo coordinates")
    f.add_argument("--step", type=int, default=1, help="use every N-th frame")

    fo = sub.add_parser("forces", help="cuff/belt/binding loads along a motion")
    fo.add_argument("model", help=".osim with exo")
    fo.add_argument("motion", help=".mot/.sto with human AND exo coordinates (e.g. from 'fit')")
    fo.add_argument("-o", "--out", required=True, help="output .sto")
    fo.add_argument("--plot", help="also save a PNG plot")
    fo.add_argument("--no-speeds", action="store_true", help="ignore damping (no finite-difference speeds)")

    args = p.parse_args(argv)
    if args.cmd == "presets":
        for name in config.list_presets():
            print(name)
    elif args.cmd == "build":
        _build(args)
    elif args.cmd == "fit":
        _fit(args)
    elif args.cmd == "forces":
        _forces(args)


if __name__ == "__main__":
    main()
