"""Local web viewer: edit an exo configuration and see it on the human model in real time.

    python -m exo_builder.viewer --model models/subject_walk_contact.osim --config full_bilateral

Backend: Python standard library HTTP server + OpenSim (one lock around every
OpenSim call, the API is not thread safe). Frontend: static/index.html with
Three.js. The viewer builds on a muscle-free copy of the human model (faster);
"Export .osim" builds on the original model.
"""
import argparse
import json
import mimetypes
import tempfile
import threading
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import opensim as osim  # before numpy (OpenSim 4.6 DLL clash)
import numpy as np

from .. import config as C
from ..analysis import coupling_loads, iterate_states, read_coordinates
from ..builder import build_exo
from ..geometry import rot_to_np, to_np
from ..kinematics import ExoFitter, exo_coordinates, fit_exo_to_motion
from .vtp import read_vtp

STATIC = Path(__file__).parent / "static"

# Named human poses (degrees). Both sides get the same values; pelvis_ty is set
# afterwards so the lowest contact sphere touches the floor ("feet on the floor").
# Sign conventions of the Rajagopal-type model: pelvis_tilt < 0 leans the trunk
# forward, ankle_angle > 0 is dorsiflexion; feet stay flat when
# ankle = knee - hip - pelvis_tilt.
POSES = {
    "standing": {},
    "sitting": {"pelvis_tilt": 0, "hip_flexion": 90, "knee_angle": 90, "ankle_angle": 0},
    "squat": {"pelvis_tilt": -20, "hip_flexion": 105, "knee_angle": 105, "ankle_angle": 20},
    "half_squat": {"pelvis_tilt": -10, "hip_flexion": 60, "knee_angle": 70, "ankle_angle": 20},
    "step_up": {"hip_flexion_r": 60, "knee_angle_r": 75, "ankle_angle_r": 15,
                       "hip_flexion_l": -5, "knee_angle_l": 5, "ankle_angle_l": 0},
}
GEOMS = {
    "Mesh": lambda g: {"type": "mesh", "file": osim.Mesh.safeDownCast(g).get_mesh_file()},
    "Cylinder": lambda g: {"type": "cylinder", "radius": osim.Cylinder.safeDownCast(g).get_radius(),
                           "half_height": osim.Cylinder.safeDownCast(g).get_half_height()},
    "Brick": lambda g: {"type": "brick", "half": to_np(osim.Brick.safeDownCast(g).get_half_lengths()).tolist()},
    "Ellipsoid": lambda g: {"type": "ellipsoid", "radii": to_np(osim.Ellipsoid.safeDownCast(g).get_radii()).tolist()},
    "Sphere": lambda g: {"type": "sphere", "radius": osim.Sphere.safeDownCast(g).get_radius()},
}


def quat(R):
    """Rotation matrix -> (x, y, z, w) quaternion."""
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(1 + t)
        return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return q


class Session:
    def __init__(self, human_model, cfg):
        self.lock = threading.RLock()
        self.human_path = str(Path(human_model).resolve())
        proc = osim.ModelProcessor(self.human_path)
        proc.append(osim.ModOpRemoveMuscles())
        tmp = Path(tempfile.mkdtemp(prefix="exo_viewer_")) / "human_viewer.osim"
        proc.process().printToXML(str(tmp))
        self.viewer_path = str(tmp)
        self.mesh_dirs = [Path(self.human_path).parent / "Geometry", Path(self.human_path).parent,
                          Path.cwd() / "Geometry"]
        self.mesh_cache = {}
        self.pose_deg = {}
        self.autofit = True
        self.ground_feet = True
        self.cfg = C.load_config(cfg)
        self.build(self.cfg)

    # ----------------------------------------------------------- building
    def build(self, cfg):
        cfg = C.load_config(cfg)
        model, info = build_exo(self.viewer_path, cfg)
        self.state = model.initSystem()
        self.model, self.info, self.cfg = model, info, cfg
        self.exo_names = set(exo_coordinates(model))
        self.fitter = ExoFitter(model, self.state)
        self.frames = []
        for comp in model.getComponentsList():
            f = osim.Frame.safeDownCast(comp)
            if f is None:
                continue
            geoms = []
            for i in range(f.getPropertyByName("attached_geometry").size()):
                g = f.get_attached_geometry(i)
                maker = GEOMS.get(g.getConcreteClassName())
                app = g.get_Appearance()
                if maker is None or not app.get_visible():
                    continue
                d = maker(g)
                d.update(color=to_np(app.get_color()).tolist(), opacity=app.get_opacity(),
                         scale=to_np(g.get_scale_factors()).tolist())
                geoms.append(d)
            if geoms:
                path = f.getAbsolutePathString()
                self.frames.append({"path": path, "exo": "/exo_" in path, "geoms": geoms, "obj": f})
        self.apply_pose()

    def coordinates(self):
        out = []
        for c in self.model.getCoordinateSet():
            name = c.getName()
            if name in self.exo_names or c.isDependent(self.state):
                continue
            rot = c.getMotionType() == 1
            k = np.degrees(1) if rot else 1.0
            out.append({"name": name, "rotational": rot, "min": c.getRangeMin() * k, "max": c.getRangeMax() * k,
                        "value": c.getValue(self.state) * k})
        return out

    def apply_pose(self):
        cs = self.model.getCoordinateSet()
        for name, v in self.pose_deg.items():
            if cs.contains(name) and name not in self.exo_names:
                c = cs.get(name)
                c.setValue(self.state, float(np.radians(v) if c.getMotionType() == 1 else v), False)
        self._update_dependent()
        if self.autofit:
            self.fitter.fit(self.state)
        if self.ground_feet and cs.contains("pelvis_ty"):
            self._put_feet_on_floor()
        self.model.realizeVelocity(self.state)

    def _put_feet_on_floor(self):
        """Shift pelvis_ty so the lowest contact sphere touches y = 0, then refit the exo."""
        self.model.realizePosition(self.state)
        lowest = np.inf
        cgs = self.model.getContactGeometrySet()
        for i in range(cgs.getSize()):
            sph = osim.ContactSphere.safeDownCast(cgs.get(i))
            if sph is not None:
                y = sph.getFrame().findStationLocationInGround(self.state, sph.get_location()).get(1)
                lowest = min(lowest, y - sph.getRadius())
        if not np.isfinite(lowest) or abs(lowest) < 1e-5:
            return
        ty = self.model.getCoordinateSet().get("pelvis_ty")
        ty.setValue(self.state, ty.getValue(self.state) - lowest, False)
        self.pose_deg["pelvis_ty"] = ty.getValue(self.state)
        if self.autofit:
            self.fitter.fit(self.state)

    def set_named_pose(self, name):
        if name not in POSES:
            raise ValueError(f"unknown pose {name!r}; options: {list(POSES)}")
        self.pose_deg = {}
        self.state = self.model.initSystem()
        self.fitter = ExoFitter(self.model, self.state)
        cs = self.model.getCoordinateSet()
        for key, v in POSES[name].items():
            names = [key] if cs.contains(key) else [f"{key}_{s}" for s in ("r", "l")]
            for n in names:
                if cs.contains(n):
                    self.pose_deg[n] = v

    def _update_dependent(self):
        """Patella-like CoordinateCouplerConstraints (dependent = f(independent))."""
        cs = self.model.getCoordinateSet()
        for con in self.model.getConstraintSet():
            cc = osim.CoordinateCouplerConstraint.safeDownCast(con)
            if cc is None:
                continue
            names = cc.getIndependentCoordinateNames()
            x = osim.Vector([cs.get(names.get(i)).getValue(self.state) for i in range(names.getSize())])
            dep = cs.get(cc.getDependentCoordinateName())
            # no getScaleFactor() in Python: read the property directly
            k = osim.PropertyHelper.getValueDouble(cc.getPropertyByName("scale_factor"))
            dep.setValue(self.state, cc.getFunction().calcValue(x) * k, False)

    # ----------------------------------------------------------- outputs
    def scene(self):
        return {"frames": [{k: f[k] for k in ("path", "exo", "geoms")} for f in self.frames],
                "coordinates": self.coordinates(), "info": self.info, "config": self.cfg,
                "pose": self.pose()}

    def pose(self, state=None):
        s = state or self.state
        transforms = {}
        for f in self.frames:
            T = f["obj"].getTransformInGround(s)
            transforms[f["path"]] = to_np(T.p()).tolist() + quat(rot_to_np(T.R()))
        loads = []
        fs = self.model.getForceSet()
        for name, d in coupling_loads(self.model, s).items():
            rec = fs.get(name).getRecordValues(s)
            f1 = osim.PhysicalFrame.safeDownCast(self.model.getComponent(
                fs.get(name).getSocket("frame1").getConnecteePath()))
            T1 = f1.getTransformInGround(s)
            loads.append({"name": name, "origin": to_np(T1.p()).tolist(),
                          "force": [rec.get(i) for i in range(3)],
                          # cuff axes in ground: anteroposterior, longitudinal, mediolateral
                          "axes": rot_to_np(T1.R()).T.tolist(),
                          **{k: float(d[k]) for k in ("F_ap", "F_axial", "F_ml", "M_ap", "M_axial", "M_ml",
                                                      "F_lat", "F_total")}})
        exo = {n: float(np.degrees(self.model.getCoordinateSet().get(n).getValue(s))
                        if self.model.getCoordinateSet().get(n).getMotionType() == 1
                        else self.model.getCoordinateSet().get(n).getValue(s)) for n in sorted(self.exo_names)}
        return {"transforms": transforms, "loads": loads, "exo_coordinates": exo,
                "coordinates": self.coordinates() if state is None else None}

    def motion(self, path, step):
        time, table, energy = fit_exo_to_motion(self.model, path, step=max(1, int(step)))
        t, data = read_coordinates(table, self.model)
        frames = [{"t": float(tt), **self.pose(s)} for tt, s in iterate_states(self.model, t, data)]
        self.state = self.model.initSystem()
        self.fitter = ExoFitter(self.model, self.state)
        self.apply_pose()
        return {"frames": frames, "energy": energy.tolist()}

    def mesh(self, name):
        if name not in self.mesh_cache:
            for d in self.mesh_dirs:
                p = d / name
                if p.exists():
                    self.mesh_cache[name] = read_vtp(p)
                    break
            else:
                self.mesh_cache[name] = None
        return self.mesh_cache[name]

    def export(self, path):
        model, info = build_exo(self.human_path, self.cfg)
        model.printToXML(str(path))
        C.save_config(self.cfg, Path(path).with_suffix(".exo.yaml"))
        return {"written": str(Path(path).resolve()), "warnings": info["warnings"]}


def make_handler(session):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body, default=float).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _json(self):
            n = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(n) or b"{}")

        def _guard(self, fn):
            try:
                with session.lock:
                    self._send(200, fn())
            except C.ConfigError as e:
                self._send(400, {"error": str(e)})
            except Exception as e:  # report to the page instead of killing the server
                traceback.print_exc()
                self._send(500, {"error": f"{type(e).__name__}: {e}"})

        def do_GET(self):
            url = urlparse(self.path)
            if url.path in ("/", "/index.html"):
                return self._send(200, (STATIC / "index.html").read_bytes(), "text/html; charset=utf-8")
            if url.path.startswith("/static/"):
                p = (STATIC / url.path[len("/static/"):]).resolve()
                if STATIC.resolve() in p.parents and p.exists():
                    ctype = mimetypes.guess_type(str(p))[0] or "application/octet-stream"
                    return self._send(200, p.read_bytes(), ctype)
                return self._send(404, {"error": "not found"})
            if url.path == "/api/scene":
                return self._guard(session.scene)
            if url.path == "/api/meta":
                return self._guard(lambda: {
                    "presets": C.list_presets(), "defaults": C.DEFAULTS, "dof_defaults": C.DOF_DEFAULTS,
                    "enums": {"mode": C.FOOT_MODES, "model": C.COUPLING_MODELS, "limits": C.LIMIT_MODES},
                    "docs": C.PARAM_DOCS, "poses": list(POSES),
                    "model": session.human_path})
            if url.path == "/api/mesh":
                name = parse_qs(url.query).get("file", [""])[0]
                return self._guard(lambda: session.mesh(Path(name).name) or {"missing": name})
            if url.path == "/api/preset":
                name = parse_qs(url.query).get("name", [""])[0]
                return self._guard(lambda: C.load_config(name))
            self._send(404, {"error": "not found"})

        def do_POST(self):
            url = urlparse(self.path)
            body = self._json()
            if url.path == "/api/build":
                def run():
                    session.build(body["config"])
                    return session.scene()
                return self._guard(run)
            if url.path == "/api/pose":
                def run():
                    session.autofit = bool(body.get("autofit", session.autofit))
                    session.ground_feet = bool(body.get("ground_feet", session.ground_feet))
                    if body.get("named"):
                        session.set_named_pose(body["named"])
                    session.pose_deg.update(body.get("pose", {}))
                    if body.get("reset"):
                        session.set_named_pose("standing")
                    session.apply_pose()
                    return session.pose()
                return self._guard(run)
            if url.path == "/api/motion":
                return self._guard(lambda: session.motion(body["path"], body.get("step", 2)))
            if url.path == "/api/save":
                def run():
                    C.save_config(session.cfg, body["path"])
                    return {"written": str(Path(body["path"]).resolve())}
                return self._guard(run)
            if url.path == "/api/export":
                return self._guard(lambda: session.export(body["path"]))
            self._send(404, {"error": "not found"})

    return Handler


def main(argv=None):
    p = argparse.ArgumentParser(description="exo_builder web viewer")
    p.add_argument("--model", default="models/subject_walk_contact.osim", help="human .osim")
    p.add_argument("--config", default="full_bilateral", help="preset name or YAML")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--no-browser", action="store_true")
    args = p.parse_args(argv)
    osim.Logger.setLevelString("warn")
    print("preparing model ...", flush=True)
    session = Session(args.model, args.config)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(session))
    url = f"http://127.0.0.1:{args.port}/"
    print(f"exo viewer ready at {url}  (Ctrl+C to stop)", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
