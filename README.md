# OpenSim-ExoGen

Opensim model generator of human body with exoskeleton. Focused on parameters of the exoskeleton.

*Leia em português: [README.pt-BR.md](README.pt-BR.md)*

## exo_builder — configurable exoskeleton for OpenSim models

Adds an exoskeleton (direct joint actuation) around an OpenSim human model from a
YAML file, models the human–exo interface (cuffs, pelvic belt, foot binding,
ground contact) and computes the interaction forces. Replaces the original
`exo_generator.py` script (kept untouched in [legacy/](legacy/README.md)).

## Layout

```
exo_builder/        package (config, builder, interaction, analysis, kinematics, CLI)
  presets/          ready-made exos in YAML
  viewer/           web viewer
examples/           moco_track_exo.py (MocoTrack with an exo)
tests/              pytest
models/             subject_walk_contact.osim (base human model) + Geometry/ (.vtp meshes)
data/walking/       gait data: MARKERS (.trc), MOCAP-IK (.mot), FORCE-MOT, SETUP-ID, SO-results
legacy/             scripts, models, setups and outputs of the old workflow (see legacy/README.md)
```

Run the commands below from the project root.

Target environment: conda `opensim46` (OpenSim 4.6, Python 3.11). Extra
dependencies: `pip install pyyaml pytest` (numpy, scipy and matplotlib come with
the environment).

> **Golden rule:** the first import of any script must be `import exo_builder`.
> On Windows with OpenSim 4.6 it works around three DLL conflicts that crash the
> process or Moco (see [Environment issues](#environment-issues)).

## Quick start

```bash
python -m exo_builder presets
python -m exo_builder build hip_bilateral --model models/subject_walk_contact.osim -o subject_hip_exo.osim
python -m exo_builder fit subject_hip_exo.osim data/walking/MOCAP-IK/walking1.mot -o walking1_exo.mot
python -m exo_builder forces subject_hip_exo.osim walking1_exo.mot -o walking1_cuffs.sto --plot cuffs.png
python -m exo_builder.viewer --model models/subject_walk_contact.osim --config full_bilateral
```

From Python:

```python
import exo_builder                       # first!
import opensim as osim
model, info = exo_builder.build_exo("models/subject_walk_contact.osim", "my_exo.yaml")
model.printToXML("human_with_exo.osim")
print(info["couplings"], info["warnings"])
```

## Describing an exo (YAML)

Every file is merged over the defaults in `exo_builder/config.py::DEFAULTS`, so you
only write what changes. The fully commented template is
[exo_builder/presets/full_bilateral.yaml](exo_builder/presets/full_bilateral.yaml).
Angles in degrees, SI elsewhere; `{s}` becomes `r`/`l`.

| Preset | What it is |
|---|---|
| `full_bilateral` | belt + hip (actuated flexion, passive adduction/rotation) + knee + ankle, both sides |
| `full_plate_contact` | same, but the foot plate carries the ground contact |
| `hip_bilateral` | belt + thighs, only hip flexion actuated |
| `hip_knee_left` | left side only, hip + knee |
| `knee_right` | right knee without belt (the thigh shell "floats", held only by its cuffs) |
| `ankle_bilateral` | shank shell + foot plate, no belt |

Editing examples:

```yaml
name: my_exo
sides: [r]                                  # unilateral
segments:
  foot: {enabled: false}                    # removes the foot (and the ankle joint)
  thigh:
    mass: 1.2
    cuffs:                                  # as many cuffs as you want
      - {name: thigh_prox, along_segment: 0.3, width: 0.06}     # 0 = hip, 1 = knee; width in m
      - name: thigh_dist
        along_segment: 0.8
        model: nonlinear
        stiffness_cubic: {anteroposterior: 2.0e6}               # N/m³
joints:
  knee:
    misalignment: [0.0, 0.02, 0.0]          # exo axis 2 cm above the anatomical knee
    dofs:
      flexion: {max_torque: 60, limits: soft}
overrides:
  l: {segments: {thigh: {mass: 1.4}}}       # different value on the left side only
```

Chain rules: `pelvis → thigh → shank → foot`. Joints exist between consecutive
enabled segments (hip, knee, ankle). A segment whose parent is disabled becomes a
**floating root** (6-DOF joint to ground) held to the human only by its cuffs.
DOFs: `enabled` (otherwise locked), `actuated` (creates `tau_exo_*`, a
`CoordinateActuator` with `optimal_force = max_torque`),
`passive_stiffness/passive_damping` (torsional spring/damper at the joint),
`limits: clamp | soft | none`. The default axis (`axis: human`) is the one of the
matching human coordinate, so a positive exo angle equals a positive human angle.

## Human–exo interface

Each coupling is a pair of frames that coincide at the default pose (zero preload),
one on the human (`<name>_human`) and one on the exo (`<name>_exo`), linked by:

* `linear` → 6-DOF `BushingForce`; `nonlinear` → `ExpressionBasedBushingForce`
  (`F = k·d + cubic·d³`, or custom expressions in `delta_x..z`/`theta_x..z`);
* `weld` → `WeldConstraint` (rigid belt or foot).

Cuff frame directions (same for cuffs, the belt `attach` and the foot `binding`):

| Key | Axis | Positive direction |
|---|---|---|
| `anteroposterior` | X | forward |
| `longitudinal` | Y | along the segment, towards the proximal joint (up) |
| `mediolateral` | Z | to the right of the body |

Coupling parameters (per direction):

| Key | Unit | Meaning |
|---|---|---|
| `stiffness_translational` | N/m | stiffness against the exo sliding relative to the limb |
| `stiffness_rotational` | N·m/rad | stiffness against the exo rotating relative to the limb (about each axis) |
| `damping_translational` | N·s/m | damping of the relative velocity |
| `damping_rotational` | N·m·s/rad | damping of the relative angular velocity |
| `stiffness_cubic` | N/m³ | `nonlinear` only: F = k·d + k3·d³ |

Cuff: `along_segment` (0–1: 0 = proximal joint, 1 = distal joint), `width` (m),
`radius` (m or `auto`), `offset` (m, in the three directions). Every key, with its
unit and description, is in `PARAM_DOCS` (`exo_builder/config.py`), which also
drives the "?" tooltips of the viewer. Files using the old short names (`at`,
`ap/axial/ml`, `translational`, `cubic`, joint `stiffness/damping`) are migrated
automatically when loaded.

Default anteroposterior values: thigh k = 3283 N/m, b = 367 N·s/m; shank
k = 5645 N/m, b = 418 N·s/m
([doi:10.1007/s11071-022-07876-8](https://doi.org/10.1007/s11071-022-07876-8)).
The others (longitudinal, mediolateral, rotational, belt, foot binding) are
engineering estimates — tune them.

Foot (`segments.foot.mode`): `weld` (legacy), `bushing` (strap; only the human
touches the floor) or `plate_contact` (the human contact spheres push on a
half-space on top of the plate, and the plate gets its own spheres against the
floor, with the same parameters).

### Interaction forces

How the force arises: each coupling links a point on the limb to a point on the
exo, coincident at the reference pose. When the exo moves or rotates relative to
the limb, the spring-damper produces **F = −K·displacement − B·velocity** (and a
moment **M = −Kr·θ − Br·ω**), expressed in the cuff directions. It acts at the
cuff centre: on the human in one direction and on the exo in the opposite one
(action and reaction).

`analysis.coupling_loads(model, state)` and `coupling_loads_over_motion(model, mot)`
return, for each coupling, what the exo applies **on the human**, in the cuff
frame: `F_ap` (anteroposterior), `F_axial` (longitudinal), `F_ml` (mediolateral),
`M_ap`, `M_axial` (twist about the limb), `M_ml`, `F_lat` (+ lateral for that
side), `F_radial` (√(F_ap²+F_ml²), perpendicular to the limb) and `F_total`, in N
and N·m.

### Exo kinematics from human kinematics

`kinematics.fit_exo_to_motion` solves, frame by frame, the exo coordinates that
minimise the elastic energy of the couplings with the human held fixed (the pose
of a massless, unactuated exo). It writes a complete `.mot` (human + exo) for
ID/SO, a Moco initial guess or force analysis. Replaces
`legacy/scripts/IK_setup_virtual_marker.py`.

## Moco

[examples/moco_track_exo.py](examples/moco_track_exo.py) builds the exo, fits its
kinematics, runs `MocoTrack` (muscles removed, reserves on human coordinates only)
and computes the cuff loads along the solution. Tested: `hip_bilateral`,
0.5–0.8 s, 0.05 s mesh → *Optimal Solution Found* in ~60 s.

Note on `legacy/scripts/exampleMocoTrack.py`: it removes the exo actuators with a
fixed index (`remove(qt)`, which removes the wrong item) and then calls
`track.setModel(modelProcessor)` with the original processor, discarding the
modified model.

## Viewer

`python -m exo_builder.viewer --model models/subject_walk_contact.osim --config full_bilateral`
opens `http://127.0.0.1:8765/`: 3D on the left; on the right, the **Exo** tab (a
form generated from the config: any new YAML key shows up by itself; the model
rebuilds on every edit), **Pose** (sliders for the human coordinates, exo fitted
automatically), **Motion** (loads a `.mot`, fits and animates it, with a plot per
coupling) and **Forces** (table). Ready-made poses in the Pose tab: standing,
sitting, squat, half squat and step-up; with "feet on the floor" the pelvis height
is adjusted so the lowest foot contact touches the floor. Every field shows its
unit and a "?" with an explanation. Force arrows have their tip at the cuff centre
(where the force acts), are drawn on top of the meshes and labelled in N; you can
show the resultant or the components in the cuff axes, and switch between the
force on the human and the reaction on the exo. It saves YAML and exports `.osim`
(built on the original model, with muscles). Three.js is loaded from jsdelivr (the
browser needs internet access).

## Tests

```bash
python -m pytest
```

Includes OpenSim API contracts (undocumented behaviours the package relies on;
they pass on 4.5 and 4.6), building every preset with zero preload, physical
responses (misalignment, nonlinear cuff, foot modes) and a dynamic test: with the
human locked, the hanging exo settles and the total coupling force equals the exo
weight (±3%).

## Environment issues

Observed on Windows with the conda `opensim46` environment (`import exo_builder`
handles all three):

1. `opensim` ships its own `libblas.dll`/`liblapack.dll`; if numpy is imported
   first, Simbody binds to MKL's same-named DLLs and `assemble()` dies with an
   access violation.
2. Another OpenSim install on the system PATH (here `C:\OpenSim 4.5\bin`, which
   also received the CoinHSL DLLs) makes the 4.6 CasADi load the 4.5 Ipopt plugin
   → *Plugin 'ipopt' is not found* (WIN32 error 127). Fix: `CASADIPATH`.
3. conda's MKL uses a `libiomp5md.dll` that is an llvm-openmp shim (forwarding to
   `libomp.dll`, found in that 4.5 folder); OpenSim 4.6's Ipopt/MUMPS need Intel's
   OpenMP → WIN32 error 182. Fix: `MKL_THREADING_LAYER=SEQUENTIAL` before
   `import opensim`.

Permanent cleanup (optional): remove the other OpenSim install from the system
PATH. HSL can still be used by pointing to its folder explicitly.

## OpenSim API pitfalls (verified)

* Adding any component invalidates the System → `human.py` snapshots the pose first.
* `safeDownCast(obj.clone())` leaves a dangling pointer → the interpreter aborts.
* Duplicate coordinate names are accepted silently (the legacy `q_rz`/`q_ry` bug) → the builder checks.
* `BushingForce.getRecordValues`: force/torque on the frame1 and frame2 bodies, in ground, torque about the **body origin**.
* `FreeJoint`: speeds ≠ coordinate derivatives and Moco rejects it → roots use a 6-DOF `CustomJoint`.
* `CustomJoint` coordinates only exist after `model.addJoint`.

## Limitations / next steps

* The exo knee is a pin; the human *walker knee* couples rotation/adduction →
  incompatibility forces (~100 N mediolateral at the shank cuff during gait) with
  the full exo. It is the physics of the model, but extra passive DOFs or other
  stiffnesses are worth studying.
* Non-sagittal, belt and binding stiffness/damping values are estimates.
* The default pose of the human model penetrates the floor; simulating with
  `plate_contact` needs to start from an equilibrium pose.
