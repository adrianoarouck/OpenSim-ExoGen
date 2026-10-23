import numpy as np

def compute_HJC(LASI, RASI, LPSI, RPSI):
    """
    Compute left and right Hip Joint Centers (HJC) using Plug-in Gait equations.
    Input markers: 3D numpy arrays in global coordinates (meters)
    Output: dict with LHJC, RHJC in meters
    """

    # Pelvis origin (midpoint between ASIS markers)
    origin = (LASI + RASI) / 2

    # Pelvis coordinate axes
    x_axis = (RASI - LASI)
    x_axis /= np.linalg.norm(x_axis)

    sacrum = (LPSI + RPSI) / 2
    y_temp = sacrum - origin
    z_axis = np.cross(x_axis, y_temp)
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)

    # Rotation matrix (pelvis -> global)
    R_pelvis = np.column_stack((x_axis, y_axis, z_axis))

    # Pelvis width (mm)
    PW = np.linalg.norm(RASI - LASI) * 1000

    # Bell regression (mm)
    offset_R = np.array([-0.24 * PW - 9.9, -0.30 * PW - 10.9, 0.33 * PW + 7.3])
    offset_L = np.array([ 0.24 * PW + 9.9, -0.30 * PW - 10.9, 0.33 * PW + 7.3])

    # Convert to meters and global frame
    RHJC = origin + R_pelvis.dot(offset_R / 1000.0)
    LHJC = origin + R_pelvis.dot(offset_L / 1000.0)

    return RHJC, LHJC

def print_transform(transform):
    R = transform.R()
    p = transform.p()
    print("Rotation matrix:")
    for i in range(3):
        print(f"  {R.get(i, 0): .4f}  {R.get(i, 1): .4f}  {R.get(i, 2): .4f}")
    print(f"Translation: [{p[0]: .4f}, {p[1]: .4f}, {p[2]: .4f}]")


def r_to_numpy(self):
    return np.array([[self.get(i, j) for j in range(3)] for i in range(3)])

class MarkerConventions:
    """
    Stateful mapping for OpenSim marker naming conventions.
    """
    def __init__(self):
        # Default to primary convention (Index 0)
        self.active_index = 0
        
        # Centralized mapping dictionary
        self._mappings = {
            # --- Pelvis & Trunk ---
            'right_ASIS': ["R.ASIS", "r.ASIS"],
            'left_ASIS':  ["L.ASIS", "L.ASIS"],
            'right_PSIS': ["R.PSIS", "r.PSIS"],
            'left_PSIS':  ["L.PSIS", "L.PSIS"],
            'S2':         ["S2", None],
            
            # --- Right Leg ---
            'right_thigh1': ["R.TH1", "r_thigh1"],
            'right_thigh2': ["R.TH2", "r_thigh2"],
            'right_thigh3': ["R.TH3", "r_thigh3"],
            'right_thigh4': ["R.TH3", "r_thigh4"],
            'right_thigh5': [None, "r_thigh5"],
            #right thig4 not present in convention 1, but we can use 3 as a proxy.
            'right_knee':   ["R.Knee", "r_knee"],
            'right_mknee':  [None, "r_mknee"],
            
            'right_shank1': ["R.SH1", "r_shank_antsup"],
            'right_shank2': ["R.SH2", "r_sh2"],
            'right_shank3': ["R.SH3", "r_sh3"],
            'right_shank4': ["R.SH4", "r_sh4"],
            
            'right_ankle':  ["R.Ankle", "r_ankle"],
            'right_mankle': [None, "r_mankle"],
            'right_toe':    ["R.Toe", "r_toe"],
            'right_MT5':    ["R.MT5", "r_5meta"],
            'right_heel':   ["R.Heel", "r_calc"],

            # --- Left Leg ---
            'left_thigh1':  ["L.TH1", "L_thigh1"],
            'left_thigh2':  ["L.TH2", "L_thigh2"],
            'left_thigh3':  ["L.TH3", "L_thigh3"],
            'left_thigh4':  ["L.TH4", "L_thigh4"],
            
            'left_knee':    ["L.Knee", "L_knee"],
            'left_mknee':   [None, "L_mknee"],
            
            'left_shank1':  ["L.SH1", "L_shank_antsup"],
            'left_shank2':  ["L.SH2", "L_sh2"],
            'left_shank3':  ["L.SH3", "L_sh3"],
            
            'left_ankle':   ["L.Ankle", "L_ankle"],
            'left_mankle':  [None, "L_mankle"],
            'left_toe':     ["L.Toe", "L_toe"],
            'left_MT5':     ["L.MT5", "L_5meta"],
            'left_heel':    ["L.Heel", "L_calc"],

            # --- Upper Body & Extras (Convention 2 Only) ---
            'C7':             [None, "C7"],
            'right_shoulder': [None, "R_Shoulder"],
            'left_shoulder':  [None, "L_Shoulder"],
            'right_sternum':  [None, "R_Sternum"],
            'left_sternum':   [None, "L_Sternum"],
            'right_HJC':      [None, "R_HJC"],
            'left_HJC':       [None, "L_HJC"],
            
            'right_elbow_med':    [None, "R_elbow_med"],
            'right_elbow_lat':    [None, "R_elbow_lat"],
            'right_wrist_radius': [None, "R_wrist_radius"],
            'right_wrist_ulna':   [None, "R_wrist_ulna"],
            
            'left_elbow_med':     [None, "L_elbow_med"],
            'left_elbow_lat':     [None, "L_elbow_lat"],
            'left_wrist_radius':  [None, "L_wrist_radius"],
            'left_wrist_ulna':    [None, "L_wrist_ulna"]
        }

    def __getattr__(self, name):
        """
        Intercepts attribute access. Returns the string for the active convention.
        """
        if name in self._mappings:
            return self._mappings[name][self.active_index]
        raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")

    def detect_convention(self, model):
        """
        Queries the OpenSim model's MarkerSet to determine the active convention.
        Modifies self.active_index.
        """
        marker_set = model.getMarkerSet()
        
        # getIndex returns -1 if the component does not exist
        if marker_set.getIndex("C7") != -1:
            self.active_index = 1
        else:
            self.active_index = 0
            
        print(f"Marker convention set to index: {self.active_index}")

    # --- Execution Workflow ---
    #import opensim as osim

    # 1. Initialize the mapper
    #markers = MarkerConventions()

    # 2. Load the model
    # model = osim.Model("path_to_your_model.osim")

    # 3. Detect and set the convention automatically
    # markers.detect_convention(model)

    # 4. Access markers dynamically without specifying the index
    # marker_name = markers.right_PSIS
    #
    # if marker_name:
    #     marker_location = model.getMarkerSet().get(marker_name).getLocationInGround(state)