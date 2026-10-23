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

# Parameter documentation: key -> (unit, short description in Portuguese).
# Shown as "?" tooltips by the viewer; keys are matched by name.
PARAM_DOCS = {
    "name": ("", "Nome do exo (vai para o arquivo e para o modelo)."),
    "sides": ("", "Lados com exo: direito (r) e/ou esquerdo (l). Um só lado = exo unilateral."),
    "segments": ("", "Partes do exo ao longo da perna: coxa → canela → pé. Desabilitar uma parte remove a junta que a liga à anterior."),
    "joints": ("", "Juntas do exo (quadril, joelho, tornozelo): graus de liberdade, atuação e desalinhamento."),
    "pelvis_unit": ("", "Cinto pélvico + mochila. Se desabilitado, a coxa do exo fica 'flutuante', presa só pelos cuffs."),
    "cuff_defaults": ("", "Valores usados por todos os cuffs; cada cuff pode sobrescrever qualquer um."),
    "appearance": ("", "Cores e transparência (só visualização)."),
    "human": ("", "Nomes dos corpos/juntas do modelo humano e estimativa do raio do membro."),
    "bodies": ("", "Nomes dos corpos humanos (coxa, canela, pé, dedos) aos quais o exo se prende."),
    "overrides": ("", "Valores diferentes para um lado só, ex.: {\"l\": {\"segments\": {\"thigh\": {\"mass\": 1.2}}}}."),
    "enabled": ("", "Liga/desliga este item."),
    "mass": ("kg", "Massa total desta parte (barra + cuffs, ou placa, ou cinto)."),
    "bar_radius": ("m", "Raio da barra lateral do exo (afeta inércia e desenho)."),
    "clearance": ("m", "Folga entre a pele e a barra lateral (ou o cinto)."),
    "cuffs": ("", "Braçadeiras que prendem o exo ao membro. Cada uma é uma mola-amortecedor 6-DOF entre o membro e o exo."),
    "along_segment": ("0–1", "Posição do cuff ao longo do segmento: 0 = junta proximal (ex.: quadril), 1 = junta distal (ex.: joelho)."),
    "width": ("m", "Largura (altura) da braçadeira ao longo do membro."),
    "radius": ("m", "Raio do membro na altura do cuff. 'auto' estima pelos marcadores/proporções."),
    "offset": ("m", "Deslocamento extra do centro, nos eixos [anteroposterior, longitudinal, mediolateral] (lado direito; espelhado no esquerdo)."),
    "model": ("", "linear = BushingForce (mola linear 6-DOF); nonlinear = mola linear + termo cúbico; weld = ligação rígida."),
    "stiffness_translational": ("N/m", "Rigidez da mola para deslocamentos do exo em relação ao membro, em cada direção do cuff."),
    "stiffness_rotational": ("N·m/rad", "Rigidez para rotações do exo em relação ao membro, em torno de cada eixo do cuff."),
    "damping_translational": ("N·s/m", "Amortecimento para velocidades relativas, em cada direção do cuff."),
    "damping_rotational": ("N·m·s/rad", "Amortecimento para velocidades angulares relativas, em torno de cada eixo."),
    "stiffness_cubic": ("N/m³", "Só no modelo nonlinear: F = k·d + k3·d³ (endurece com a deformação)."),
    "expressions": ("", "Avançado (nonlinear): expressões próprias Fx..Fz, Mx..Mz usando delta_x..z (m) e theta_x..z (rad)."),
    "anteroposterior": ("", "Direção X do cuff: frente (+) / trás."),
    "longitudinal": ("", "Direção Y do cuff: ao longo do segmento, + em direção à junta proximal (para cima)."),
    "mediolateral": ("", "Direção Z do cuff: lateral; + aponta para a direita do corpo."),
    "attach": ("", "Ligação do cinto à pelve (mesmos parâmetros de um cuff)."),
    "belt": ("", "Geometria do cinto."),
    "height": ("m", "Altura do cinto."),
    "backpack": ("", "Mochila (bateria/motores), presa ao cinto."),
    "size": ("m", "Dimensões [profundidade, altura, largura]."),
    "foot": ("", "Placa do pé do exo."),
    "mode": ("", "weld = presa rígida ao pé; bushing = amarração elástica (só o humano toca o chão); plate_contact = a placa toca o chão e o pé apoia na placa."),
    "plate": ("", "Dimensões da placa do pé."),
    "thickness": ("m", "Espessura da placa."),
    "length": ("m", "Comprimento da placa ('auto' = pé + margem)."),
    "margin": ("m", "Margem da placa além do pé (quando auto)."),
    "binding": ("", "Amarração pé ↔ placa (mesmos parâmetros de um cuff)."),
    "misalignment": ("m", "Centro da junta do exo menos o centro anatômico, [anteroposterior, vertical, lateral] no frame da pelve (lado direito; espelhado no esquerdo)."),
    "dofs": ("", "Graus de liberdade da junta (flexão, adução, rotação...)."),
    "actuated": ("", "Cria um atuador de torque direto nesta coordenada (tau_exo_...)."),
    "max_torque": ("N·m", "Torque máximo do atuador (controle ±1 × este valor)."),
    "range": ("graus", "Limites [mín, máx] do ângulo."),
    "default": ("graus", "Ângulo inicial."),
    "limits": ("", "clamp = limite rígido; soft = força de limite suave (CoordinateLimitForce, melhor no Moco); none = sem limite."),
    "limit_stiffness": ("N·m/grau", "Rigidez do limite suave (limits: soft)."),
    "limit_damping": ("N·m·s/grau", "Amortecimento do limite suave."),
    "passive_stiffness": ("N·m/rad", "Mola torsional passiva na junta (0 = sem mola)."),
    "passive_damping": ("N·m·s/rad", "Amortecedor torsional passivo na junta."),
    "human_coord": ("", "Coordenada humana correspondente (define o eixo e o sentido positivo)."),
    "axis": ("", "'human' = mesmo eixo da coordenada humana; ou [x, y, z] no frame da pelve (lado direito)."),
    "color_r": ("", "Cor do lado direito."), "color_l": ("", "Cor do lado esquerdo."),
    "color_pelvis": ("", "Cor do cinto/mochila."), "cuff_opacity": ("0–1", "Opacidade das braçadeiras."),
    "radius_ratio": ("fração", "Raio do membro / comprimento do segmento [proximal, distal] quando não há marcadores."),
    "use_markers": ("", "Usar marcadores do joelho/tornozelo para estimar o raio do membro."),
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
