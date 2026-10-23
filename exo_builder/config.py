"""Exoskeleton configuration: defaults, YAML loading/saving, merging and validation.

A configuration is a plain nested dict (easy to edit from YAML, JSON or the web
viewer). Every user file is deep-merged over DEFAULTS, so a preset only has to
list what differs from the full bilateral exo. Conventions:

* Angles in the file are in degrees; the builder converts to radians. Other
  units are SI and listed per parameter in PARAM_DOCS (also shown as tooltips
  in the viewer).
* Coupling (cuff/belt/binding) directions follow the coupling frame:
  ``anteroposterior`` (X, + forward), ``longitudinal`` (Y, along the segment,
  + towards the proximal joint / up) and ``mediolateral`` (Z, + to the right).
* ``{s}`` inside a string is replaced by the side letter (``r`` or ``l``).
* ``overrides: {l: {...}}`` is deep-merged for that side only.
* Files written with the old short names (at, ap/axial/ml, translational, ...)
  are migrated automatically (see LEGACY_KEYS).
"""
import copy
import re
from pathlib import Path

import yaml

SEGMENTS = ("thigh", "shank", "foot")
JOINT_OF = {"thigh": "hip", "shank": "knee", "foot": "ankle"}
PARENT_OF = {"thigh": "pelvis", "shank": "thigh", "foot": "shank"}
FOOT_MODES = ("weld", "bushing", "plate_contact")
COUPLING_MODELS = ("linear", "nonlinear", "weld")
LIMIT_MODES = ("clamp", "soft", "none")
EXPRESSION_VARS = ("theta_x", "theta_y", "theta_z", "delta_x", "delta_y", "delta_z")
DIRECTIONS = ("anteroposterior", "longitudinal", "mediolateral")  # coupling frame X, Y, Z
COUPLING_KEYS = ("stiffness_translational", "stiffness_rotational",
                 "damping_translational", "damping_rotational")


def _dirs(ap, lon, ml):
    return {"anteroposterior": ap, "longitudinal": lon, "mediolateral": ml}


# Literature values for the linear cuff model (sagittal = anteroposterior):
#   thigh k = 3283 N/m, b = 367 N.s/m ; shank k = 5645 N/m, b = 418 N.s/m
#   https://doi.org/10.1007/s11071-022-07876-8
# Non-linear (cubic) soft-tissue models: https://doi.org/10.1109/TOH.2024.3375295
# Longitudinal, mediolateral and rotational values are engineering estimates: tune them.
DEFAULTS = {
    "name": "full_bilateral",
    "sides": ["r", "l"],
    "human": {
        "pelvis": "pelvis",
        "bodies": {"thigh": "femur_{s}", "shank": "tibia_{s}", "foot": "calcn_{s}", "toes": "toes_{s}"},
        "joints": {"hip": "hip_{s}", "knee": "walker_knee_{s}", "ankle": "ankle_{s}"},
        "floor": "floor",
        "radius_ratio": {"thigh": [0.20, 0.13], "shank": [0.12, 0.07]},
        "use_markers": True,
    },
    "appearance": {"color_r": [0.0, 0.8, 0.8], "color_l": [0.8, 0.0, 0.8],
                   "color_pelvis": [1.0, 0.85, 0.0], "cuff_opacity": 0.35},
    "pelvis_unit": {
        "enabled": True,
        "mass": 2.0,
        "belt": {"offset": [-0.02, 0.0, 0.0], "height": 0.06, "clearance": 0.03},
        "backpack": {"enabled": True, "mass": 3.0, "size": [0.10, 0.30, 0.25],
                     "offset": [-0.16, 0.12, 0.0]},
        "attach": {
            "model": "linear",
            "stiffness_translational": _dirs(20000.0, 20000.0, 20000.0),
            "stiffness_rotational": _dirs(300.0, 300.0, 300.0),
            "damping_translational": _dirs(300.0, 300.0, 300.0),
            "damping_rotational": _dirs(5.0, 5.0, 5.0),
        },
    },
    "cuff_defaults": {
        "along_segment": 0.5,
        "width": 0.07,
        "radius": "auto",
        "offset": [0.0, 0.0, 0.0],
        "model": "linear",
        "stiffness_translational": _dirs(3283.0, 1000.0, 3283.0),
        "stiffness_rotational": _dirs(30.0, 5.0, 30.0),
        "damping_translational": _dirs(367.0, 100.0, 367.0),
        "damping_rotational": _dirs(1.0, 0.2, 1.0),
        "stiffness_cubic": _dirs(0.0, 0.0, 0.0),
        "expressions": {},
    },
    "segments": {
        "thigh": {
            "enabled": True,
            "mass": 1.5,
            "bar_radius": 0.012,
            "clearance": 0.015,
            "cuffs": [{"name": "thigh_cuff", "along_segment": 0.5, "width": 0.08}],
        },
        "shank": {
            "enabled": True,
            "mass": 1.0,
            "bar_radius": 0.010,
            "clearance": 0.015,
            "cuffs": [{"name": "shank_cuff", "along_segment": 0.4, "width": 0.06,
                       "stiffness_translational": {"anteroposterior": 5645.0, "mediolateral": 5645.0},
                       "damping_translational": {"anteroposterior": 418.0, "mediolateral": 418.0}}],
        },
        "foot": {
            "enabled": True,
            "mass": 0.6,
            "mode": "bushing",
            "plate": {"thickness": 0.008, "length": "auto", "width": "auto", "margin": 0.01},
            "binding": {
                "model": "linear",
                "stiffness_translational": _dirs(50000.0, 50000.0, 50000.0),
                "stiffness_rotational": _dirs(200.0, 200.0, 200.0),
                "damping_translational": _dirs(500.0, 500.0, 500.0),
                "damping_rotational": _dirs(5.0, 5.0, 5.0),
            },
        },
    },
    "joints": {
        "hip": {
            "misalignment": [0.0, 0.0, 0.0],
            "dofs": {
                "flexion": {"enabled": True, "actuated": True, "max_torque": 100.0,
                            "range": [-40.0, 120.0], "human_coord": "hip_flexion_{s}", "axis": "human"},
                "adduction": {"enabled": True, "actuated": False, "range": [-30.0, 30.0],
                              "human_coord": "hip_adduction_{s}", "axis": "human"},
                "rotation": {"enabled": True, "actuated": False, "range": [-40.0, 40.0],
                             "human_coord": "hip_rotation_{s}", "axis": "human"},
            },
        },
        "knee": {
            "misalignment": [0.0, 0.0, 0.0],
            "dofs": {
                "flexion": {"enabled": True, "actuated": True, "max_torque": 80.0,
                            "range": [-10.0, 120.0], "human_coord": "knee_angle_{s}", "axis": "human"},
            },
        },
        "ankle": {
            "misalignment": [0.0, 0.0, 0.0],
            "dofs": {
                "flexion": {"enabled": True, "actuated": True, "max_torque": 60.0,
                            "range": [-40.0, 30.0], "human_coord": "ankle_angle_{s}", "axis": "human"},
                "inversion": {"enabled": False, "actuated": False, "range": [-20.0, 20.0],
                              "human_coord": "subtalar_angle_{s}", "axis": [1.0, 0.0, 0.0]},
            },
        },
    },
    "overrides": {},
}

DOF_DEFAULTS = {"enabled": True, "actuated": False, "max_torque": 50.0, "range": [-90.0, 90.0],
                "default": 0.0, "limits": "clamp", "limit_stiffness": 20.0, "limit_damping": 0.05,
                "passive_stiffness": 0.0, "passive_damping": 0.0, "human_coord": None, "axis": "human"}

# Fallback joint axes (right side, pelvis frame) used when the human coordinate
# does not exist; the left side is mirrored as a pseudo-vector (-x, -y, z).
FALLBACK_AXES = {"flexion": [0.0, 0.0, 1.0], "adduction": [1.0, 0.0, 0.0],
                 "rotation": [0.0, 1.0, 0.0], "inversion": [1.0, 0.0, 0.0]}

# Parameter documentation: key -> (unit, short description).
# Shown as "?" tooltips by the viewer; keys are matched by name.
PARAM_DOCS = {
    "name": ("", "Name of the exo (written to the file and to the model)."),
    "sides": ("", "Sides with an exo: right (r) and/or left (l). A single side = unilateral exo."),
    "segments": ("", "Exo parts along the leg: thigh -> shank -> foot. Disabling a part removes the joint that links it to the previous one."),
    "joints": ("", "Exo joints (hip, knee, ankle): degrees of freedom, actuation and misalignment."),
    "pelvis_unit": ("", "Pelvic belt + backpack. If disabled, the exo thigh 'floats', held only by its cuffs."),
    "cuff_defaults": ("", "Values used by every cuff; each cuff may override any of them."),
    "appearance": ("", "Colours and transparency (visualisation only)."),
    "human": ("", "Names of the human model bodies/joints and limb radius estimation."),
    "bodies": ("", "Names of the human bodies (thigh, shank, foot, toes) the exo attaches to."),
    "overrides": ("", "Values for one side only, e.g. {\"l\": {\"segments\": {\"thigh\": {\"mass\": 1.2}}}}."),
    "enabled": ("", "Turns this item on/off."),
    "mass": ("kg", "Total mass of this part (bar + cuffs, plate, or belt)."),
    "bar_radius": ("m", "Radius of the exo lateral bar (affects inertia and drawing)."),
    "clearance": ("m", "Gap between the skin and the lateral bar (or the belt)."),
    "cuffs": ("", "Straps holding the exo to the limb. Each one is a 6-DOF spring-damper between the limb and the exo."),
    "along_segment": ("0-1", "Cuff position along the segment: 0 = proximal joint (e.g. hip), 1 = distal joint (e.g. knee)."),
    "width": ("m", "Width (height) of the cuff along the limb."),
    "radius": ("m", "Limb radius at the cuff. 'auto' estimates it from markers/proportions."),
    "offset": ("m", "Extra shift of the centre along [anteroposterior, longitudinal, mediolateral] (right side; mirrored on the left)."),
    "model": ("", "linear = BushingForce (linear 6-DOF spring); nonlinear = linear spring + cubic term; weld = rigid link."),
    "stiffness_translational": ("N/m", "Spring stiffness against the exo sliding relative to the limb, in each cuff direction."),
    "stiffness_rotational": ("N·m/rad", "Stiffness against the exo rotating relative to the limb, about each cuff axis."),
    "damping_translational": ("N·s/m", "Damping of the relative velocity, in each cuff direction."),
    "damping_rotational": ("N·m·s/rad", "Damping of the relative angular velocity, about each axis."),
    "stiffness_cubic": ("N/m³", "Nonlinear model only: F = k·d + k3·d³ (stiffens with deformation)."),
    "expressions": ("", "Advanced (nonlinear): custom expressions Fx..Fz, Mx..Mz using delta_x..z (m) and theta_x..z (rad)."),
    "anteroposterior": ("", "Cuff X direction: forward (+) / backward."),
    "longitudinal": ("", "Cuff Y direction: along the segment, + towards the proximal joint (up)."),
    "mediolateral": ("", "Cuff Z direction: sideways; + points to the right of the body."),
    "attach": ("", "Belt-to-pelvis link (same parameters as a cuff)."),
    "belt": ("", "Belt geometry."),
    "height": ("m", "Belt height."),
    "backpack": ("", "Backpack (battery/motors), attached to the belt."),
    "size": ("m", "Dimensions [depth, height, width]."),
    "foot": ("", "Exo foot plate."),
    "mode": ("", "weld = rigidly attached to the foot; bushing = elastic strap (only the human touches the floor); plate_contact = the plate touches the floor and the foot rests on the plate."),
    "plate": ("", "Foot plate dimensions."),
    "thickness": ("m", "Plate thickness."),
    "length": ("m", "Plate length ('auto' = foot + margin)."),
    "margin": ("m", "Plate margin beyond the foot (when auto)."),
    "binding": ("", "Foot <-> plate strap (same parameters as a cuff)."),
    "misalignment": ("m", "Exo joint centre minus anatomical centre, [anteroposterior, vertical, lateral] in the pelvis frame (right side; mirrored on the left)."),
    "dofs": ("", "Joint degrees of freedom (flexion, adduction, rotation...)."),
    "actuated": ("", "Creates a direct torque actuator on this coordinate (tau_exo_...)."),
    "max_torque": ("N·m", "Maximum actuator torque (control ±1 × this value)."),
    "range": ("deg", "Angle limits [min, max]."),
    "default": ("deg", "Initial angle."),
    "limits": ("", "clamp = hard limit; soft = smooth limit force (CoordinateLimitForce, better for Moco); none = no limit."),
    "limit_stiffness": ("N·m/deg", "Soft limit stiffness (limits: soft)."),
    "limit_damping": ("N·m·s/deg", "Soft limit damping."),
    "passive_stiffness": ("N·m/rad", "Passive torsional spring at the joint (0 = none)."),
    "passive_damping": ("N·m·s/rad", "Passive torsional damper at the joint."),
    "human_coord": ("", "Matching human coordinate (defines the axis and the positive direction)."),
    "axis": ("", "'human' = same axis as the human coordinate; or [x, y, z] in the pelvis frame (right side)."),
    "color_r": ("", "Right side colour."), "color_l": ("", "Left side colour."),
    "color_pelvis": ("", "Belt/backpack colour."), "cuff_opacity": ("0-1", "Cuff opacity."),
    "radius_ratio": ("fraction", "Limb radius / segment length [proximal, distal] when there are no markers."),
    "use_markers": ("", "Use knee/ankle markers to estimate the limb radius."),
}

# Old short names -> new names (applied when loading any config).
LEGACY_KEYS = {"at": "along_segment", "translational": "stiffness_translational",
               "rotational": "stiffness_rotational", "translational_damping": "damping_translational",
               "rotational_damping": "damping_rotational", "cubic": "stiffness_cubic",
               "ap": "anteroposterior", "axial": "longitudinal", "ml": "mediolateral",
               "stiffness": "passive_stiffness", "damping": "passive_damping"}


class ConfigError(ValueError):
    pass


def migrate(obj):
    """Rename legacy keys recursively (returns a new object)."""
    if isinstance(obj, list):
        return [migrate(v) for v in obj]
    if isinstance(obj, dict):
        return {LEGACY_KEYS.get(k, k) if not (k == "model" or k in PARAM_DOCS) else k: migrate(v)
                for k, v in obj.items()}
    return obj


def deep_merge(base, extra):
    """Return a copy of ``base`` updated recursively by ``extra`` (lists are replaced)."""
    out = copy.deepcopy(base)
    for key, val in (extra or {}).items():
        if isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], val)
        else:
            out[key] = copy.deepcopy(val)
    return out


PRESET_DIR = Path(__file__).parent / "presets"


def load_config(source=None):
    """Load a config from a YAML path, preset name, dict, or None (defaults). Returns a full dict."""
    if source is None:
        user = {}
    elif isinstance(source, dict):
        user = source
    else:
        path = Path(source)
        if not path.exists() and (PRESET_DIR / path).exists():
            path = PRESET_DIR / path
        if not path.exists() and (PRESET_DIR / f"{path}.yaml").exists():
            path = PRESET_DIR / f"{path}.yaml"
        with open(path, "r", encoding="utf-8") as f:
            user = yaml.safe_load(f) or {}
    user = migrate(user)
    if isinstance(user.get("pelvis_unit", {}).get("attach", {}).get("model"), str) and \
            user["pelvis_unit"]["attach"]["model"] == "bushing":
        user["pelvis_unit"]["attach"]["model"] = "linear"
    cfg = deep_merge(DEFAULTS, user)
    validate(cfg)
    return cfg


def save_config(cfg, path):
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)


def list_presets():
    return sorted(p.stem for p in PRESET_DIR.glob("*.yaml"))


def fill_side(obj, side):
    """Recursively replace ``{s}`` by the side letter in every string."""
    if isinstance(obj, str):
        return obj.replace("{s}", side)
    if isinstance(obj, list):
        return [fill_side(v, side) for v in obj]
    if isinstance(obj, dict):
        return {k: fill_side(v, side) for k, v in obj.items()}
    return obj


def side_view(cfg, side):
    """Config as seen by one side: overrides merged and ``{s}`` substituted."""
    merged = deep_merge(cfg, migrate(cfg.get("overrides", {}).get(side, {})))
    return fill_side(merged, side)


def cuff_spec(cfg, cuff):
    """A cuff entry merged over ``cuff_defaults``."""
    return deep_merge(cfg["cuff_defaults"], migrate(cuff))


def dof_spec(dof):
    return deep_merge(DOF_DEFAULTS, migrate(dof))


def enabled_segments(view):
    return [seg for seg in SEGMENTS if view["segments"][seg].get("enabled", True)]


def _check_axes(where, d, errors):
    for key in DIRECTIONS:
        if key not in d:
            errors.append(f"{where}: missing '{key}'")
        elif not isinstance(d[key], (int, float)) or d[key] < 0:
            errors.append(f"{where}.{key} must be a number >= 0 (got {d[key]!r})")


def _check_coupling(where, c, errors, allow_weld=True):
    model = c.get("model", "linear")
    allowed = COUPLING_MODELS + ("bushing",) if allow_weld else ("linear", "nonlinear", "bushing")
    if model not in allowed:
        errors.append(f"{where}.model must be one of {allowed} (got {model!r})")
    if model == "weld":
        return
    for key in COUPLING_KEYS:
        _check_axes(f"{where}.{key}", c.get(key, {}), errors)
    for key, expr in (c.get("expressions") or {}).items():
        if key not in ("Fx", "Fy", "Fz", "Mx", "My", "Mz"):
            errors.append(f"{where}.expressions: unknown key {key!r} (use Fx..Fz, Mx..Mz)")
        names = set(re.findall(r"[A-Za-z_][A-Za-z_0-9]*", str(expr)))
        unknown = names - set(EXPRESSION_VARS) - {"sin", "cos", "tan", "exp", "log", "sqrt", "abs",
                                                    "min", "max", "step", "delta", "square", "cube"}
        if unknown:
            # Lepton only knows theta_x..z / delta_x..z; anything else fails at
            # realizeDynamics() with "No value specified for variable ...".
            errors.append(f"{where}.expressions.{key}: unknown variables {sorted(unknown)}; "
                          f"allowed: {EXPRESSION_VARS}")


def validate(cfg):
    """Raise ConfigError listing every problem found."""
    errors = []
    sides = cfg.get("sides", [])
    if not sides or any(s not in ("r", "l") for s in sides) or len(set(sides)) != len(sides):
        errors.append(f"sides must be a non-empty subset of ['r', 'l'] (got {sides!r})")
    for side in [s for s in sides if s in ("r", "l")] or ["r"]:
        view = side_view(cfg, side)
        segs = enabled_segments(view)
        if not segs and not view["pelvis_unit"]["enabled"]:
            errors.append("nothing to build: pelvis_unit and all segments are disabled")
        for seg in SEGMENTS:
            spec = view["segments"][seg]
            where = f"segments.{seg}"
            if spec.get("enabled", True) and not spec.get("mass", 0) > 0:
                errors.append(f"{where}.mass must be > 0 (massless free bodies break the dynamics)")
            for i, cuff in enumerate(spec.get("cuffs", []) if seg != "foot" else []):
                c = cuff_spec(view, cuff)
                if not 0.0 <= c["along_segment"] <= 1.0:
                    errors.append(f"{where}.cuffs[{i}].along_segment must be in [0, 1]")
                if c["radius"] != "auto" and not (isinstance(c["radius"], (int, float)) and c["radius"] > 0):
                    errors.append(f"{where}.cuffs[{i}].radius must be 'auto' or > 0")
                _check_coupling(f"{where}.cuffs[{i}]", c, errors, allow_weld=False)
        foot = view["segments"]["foot"]
        if foot.get("enabled", True):
            if foot.get("mode") not in FOOT_MODES:
                errors.append(f"segments.foot.mode must be one of {FOOT_MODES}")
            elif foot["mode"] != "weld":
                _check_coupling("segments.foot.binding", foot["binding"], errors, allow_weld=False)
        pu = view["pelvis_unit"]
        if pu["enabled"]:
            if not pu.get("mass", 0) > 0:
                errors.append("pelvis_unit.mass must be > 0")
            _check_coupling("pelvis_unit.attach", pu["attach"], errors)
        for jname, joint in view["joints"].items():
            if len(joint.get("misalignment", [])) != 3:
                errors.append(f"joints.{jname}.misalignment must have 3 values")
            for dname, dof in joint.get("dofs", {}).items():
                d = dof_spec(dof)
                where = f"joints.{jname}.dofs.{dname}"
                lo, hi = d["range"]
                if lo >= hi:
                    errors.append(f"{where}.range must be [min, max] with min < max")
                if not lo <= d["default"] <= hi:
                    errors.append(f"{where}.default must lie inside range")
                if d["limits"] not in LIMIT_MODES:
                    errors.append(f"{where}.limits must be one of {LIMIT_MODES}")
                if d["actuated"] and not d["max_torque"] > 0:
                    errors.append(f"{where}.max_torque must be > 0 when actuated")
                if d["axis"] != "human" and (not isinstance(d["axis"], list) or len(d["axis"]) != 3):
                    errors.append(f"{where}.axis must be 'human' or [x, y, z]")
    if errors:
        raise ConfigError("Invalid exo configuration:\n  - " + "\n  - ".join(dict.fromkeys(errors)))
    return True
