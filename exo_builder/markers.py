"""Marker naming conventions (migrated from aid_functions.MarkerConventions).

Markers are optional: the builder only uses them to refine limb radii. A model
without any known convention simply falls back to anthropometric ratios.
"""
import numpy as np

# key -> [convention 0 (Plug-in Gait, "R.Knee"), convention 1 ("r_knee")]
MAPPINGS = {
    "right_ASIS": ["R.ASIS", "r.ASIS"], "left_ASIS": ["L.ASIS", "L.ASIS"],
    "right_PSIS": ["R.PSIS", "r.PSIS"], "left_PSIS": ["L.PSIS", "L.PSIS"],
    "S2": ["S2", None], "C7": [None, "C7"],
    "right_knee": ["R.Knee", "r_knee"], "left_knee": ["L.Knee", "L_knee"],
    "right_mknee": [None, "r_mknee"], "left_mknee": [None, "L_mknee"],
    "right_ankle": ["R.Ankle", "r_ankle"], "left_ankle": ["L.Ankle", "L_ankle"],
    "right_mankle": [None, "r_mankle"], "left_mankle": [None, "L_mankle"],
    "right_toe": ["R.Toe", "r_toe"], "left_toe": ["L.Toe", "L_toe"],
    "right_MT5": ["R.MT5", "r_5meta"], "left_MT5": ["L.MT5", "L_5meta"],
    "right_heel": ["R.Heel", "r_calc"], "left_heel": ["L.Heel", "L_calc"],
    "right_thigh1": ["R.TH1", "r_thigh1"], "right_thigh2": ["R.TH2", "r_thigh2"],
    "right_thigh3": ["R.TH3", "r_thigh3"], "left_thigh1": ["L.TH1", "L_thigh1"],
    "left_thigh2": ["L.TH2", "L_thigh2"], "left_thigh3": ["L.TH3", "L_thigh3"],
    "right_shank1": ["R.SH1", "r_shank_antsup"], "left_shank1": ["L.SH1", "L_shank_antsup"],
}
SIDE_PREFIX = {"r": "right", "l": "left"}


class MarkerConventions:
    """Resolve logical marker keys ('right_knee') to the names used by a model."""

    def __init__(self, model=None):
        self.active_index = None
        self._names = set()
        if model is not None:
            self.detect_convention(model)

    def detect_convention(self, model):
        ms = model.getMarkerSet()
        self._names = {ms.get(i).getName() for i in range(ms.getSize())}
        hits = [sum(1 for names in MAPPINGS.values() if names[k] in self._names) for k in range(2)]
        self.active_index = int(np.argmax(hits)) if max(hits) > 0 else None
        return self.active_index

    def name(self, key):
        if self.active_index is None or key not in MAPPINGS:
            return None
        n = MAPPINGS[key][self.active_index]
        return n if n in self._names else None

    def side(self, side, key):
        return self.name(f"{SIDE_PREFIX[side]}_{key}")

    def __getattr__(self, key):
        # Backwards compatible: markers.right_knee -> "R.Knee"
        if key in MAPPINGS:
            return self.name(key)
        raise AttributeError(key)


def compute_HJC(LASI, RASI, LPSI, RPSI):
    """Hip joint centres (Plug-in Gait / Bell regression), inputs in metres (ground)."""
    origin = (LASI + RASI) / 2
    x_axis = (RASI - LASI) / np.linalg.norm(RASI - LASI)
    sacrum = (LPSI + RPSI) / 2
    z_axis = np.cross(x_axis, sacrum - origin)
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    R_pelvis = np.column_stack((x_axis, y_axis, z_axis))
    PW = np.linalg.norm(RASI - LASI) * 1000
    offset_R = np.array([-0.24 * PW - 9.9, -0.30 * PW - 10.9, 0.33 * PW + 7.3])
    offset_L = np.array([0.24 * PW + 9.9, -0.30 * PW - 10.9, 0.33 * PW + 7.3])
    return origin + R_pelvis.dot(offset_R / 1000.0), origin + R_pelvis.dot(offset_L / 1000.0)
