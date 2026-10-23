// Exo Builder viewer: Three.js scene on the left, config editor / pose / motion / loads on the right.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const $ = (id) => document.getElementById(id);
const state = { config: null, meta: null, frames: {}, scene: null, motion: null, playing: false, frameIdx: 0 };

// ------------------------------------------------------------------ API
async function api(path, body) {
  const res = await fetch(path, body === undefined ? {} :
    { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  const data = await res.json();
  if (!res.ok || data.error) throw new Error(data.error || res.statusText);
  return data;
}
function setStatus(text) { $("status").textContent = text; }
function showError(err) {
  const box = $("error");
  if (!err) { box.hidden = true; return; }
  box.hidden = false;
  box.textContent = String(err.message || err);
}

// ------------------------------------------------------------------ 3D scene
const renderer = new THREE.WebGLRenderer({ canvas: $("canvas"), antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
const scene3 = new THREE.Scene();
const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
scene3.background = new THREE.Color(dark ? 0x14171c : 0xf4f5f7);
const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
camera.position.set(1.8, 1.3, 2.2);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.7, 0);
scene3.add(new THREE.HemisphereLight(0xffffff, 0x667085, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(2, 4, 3);
scene3.add(sun);
const grid = new THREE.GridHelper(4, 40, dark ? 0x3a4250 : 0xb8c0cc, dark ? 0x262b33 : 0xdde1e8);
scene3.add(grid);
const root = new THREE.Group();
const arrows = new THREE.Group();
scene3.add(root, arrows);

function resize() {
  const c = renderer.domElement, w = c.clientWidth, h = c.clientHeight;
  if (c.width !== w * devicePixelRatio || c.height !== h * devicePixelRatio) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}
(function loop() {
  resize();
  controls.update();
  renderer.render(scene3, camera);
  requestAnimationFrame(loop);
})();

const meshCache = new Map();
async function meshGeometry(file) {
  if (!meshCache.has(file)) {
    meshCache.set(file, api(`/api/mesh?file=${encodeURIComponent(file)}`).then((d) => {
      if (d.missing) return null;
      const g = new THREE.BufferGeometry();
      g.setAttribute("position", new THREE.Float32BufferAttribute(d.positions, 3));
      g.setIndex(d.indices);
      if (d.normals) g.setAttribute("normal", new THREE.Float32BufferAttribute(d.normals, 3));
      else g.computeVertexNormals();
      return g;
    }));
  }
  return meshCache.get(file);
}

function material(geom, exo) {
  const humanOpacity = parseFloat($("humanOpacity").value);
  const color = exo ? new THREE.Color(...geom.color) : new THREE.Color(0xe8dcc6);
  const opacity = exo ? geom.opacity : humanOpacity;
  return new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: exo ? 0.2 : 0,
    transparent: opacity < 1, opacity, depthWrite: opacity >= 1 });
}

async function buildScene(scene) {
  root.clear();
  state.frames = {};
  for (const f of scene.frames) {
    const group = new THREE.Group();
    group.userData = { exo: f.exo };
    for (const g of f.geoms) {
      let geometry = null;
      if (g.type === "mesh") geometry = await meshGeometry(g.file);
      else if (g.type === "cylinder") geometry = new THREE.CylinderGeometry(g.radius, g.radius, 2 * g.half_height, 28, 1, g.opacity < 1);
      else if (g.type === "brick") geometry = new THREE.BoxGeometry(2 * g.half[0], 2 * g.half[1], 2 * g.half[2]);
      else if (g.type === "ellipsoid") { geometry = new THREE.SphereGeometry(1, 28, 16); geometry.scale(...g.radii); }
      else if (g.type === "sphere") geometry = new THREE.SphereGeometry(g.radius, 20, 12);
      if (!geometry) continue;
      const mesh = new THREE.Mesh(geometry, material(g, f.exo));
      mesh.scale.set(...g.scale);
      mesh.userData.exo = f.exo;
      group.add(mesh);
    }
    root.add(group);
    state.frames[f.path] = group;
  }
  applyPose(scene.pose);
}

function applyPose(pose) {
  for (const [path, t] of Object.entries(pose.transforms)) {
    const g = state.frames[path];
    if (!g) continue;
    g.position.set(t[0], t[1], t[2]);
    g.quaternion.set(t[3], t[4], t[5], t[6]);
  }
  drawArrows(pose.loads);
  fillLoadsTable(pose.loads);
  fillExoCoords(pose.exo_coordinates);
}

function drawArrows(loads) {
  arrows.clear();
  if (!$("showArrows").checked) return;
  const scale = parseFloat($("arrowScale").value);
  for (const l of loads) {
    const f = new THREE.Vector3(...l.force);
    const mag = f.length();
    if (mag < 0.5) continue;
    const t = Math.min(mag / 300, 1);
    const color = new THREE.Color().setHSL(0.33 * (1 - t), 0.85, 0.5);
    arrows.add(new THREE.ArrowHelper(f.clone().normalize(), new THREE.Vector3(...l.origin),
      Math.max(mag * scale, 0.02), color, 0.025, 0.015));
  }
}

$("showHuman").onchange = () => root.traverse((o) => { if (o.isMesh && !o.userData.exo) o.visible = $("showHuman").checked; });
$("humanOpacity").oninput = () => root.traverse((o) => {
  if (o.isMesh && !o.userData.exo) {
    const v = parseFloat($("humanOpacity").value);
    o.material.opacity = v; o.material.transparent = v < 1; o.material.depthWrite = v >= 1;
  }
});
$("showArrows").onchange = () => drawArrows(currentLoads());
$("arrowScale").oninput = () => drawArrows(currentLoads());
let lastLoads = [];
function currentLoads() { return lastLoads; }

// ------------------------------------------------------------------ tables
const fmt = (v, d = 1) => (Math.abs(v) < 0.05 && d === 1 ? "0.0" : v.toFixed(d));
function fillLoadsTable(loads) {
  lastLoads = loads;
  const rows = loads.map((l) => `<tr><td>${l.name}</td><td>${fmt(l.F_ap)}</td><td>${fmt(l.F_axial)}</td>
    <td>${fmt(l.F_ml)}</td><td>${fmt(l.F_total)}</td><td>${fmt(l.M_axial, 2)}</td></tr>`).join("");
  $("loadsTable").innerHTML = `<tr><th>acoplamento</th><th>F_ap</th><th>F_axial</th><th>F_ml</th><th>|F| N</th><th>M_ax</th></tr>${rows}`;
}
function fillExoCoords(exo) {
  $("exoCoords").innerHTML = Object.entries(exo).map(([k, v]) => `<tr><td>${k}</td><td>${v.toFixed(2)}</td></tr>`).join("");
}

// ------------------------------------------------------------------ config form
const ENUM_KEYS = { mode: "mode", model: "model", limits: "limits" };
let rebuildTimer = null;
function scheduleRebuild() {
  clearTimeout(rebuildTimer);
  setStatus("alterado…");
  rebuildTimer = setTimeout(rebuild, 400);
}
async function rebuild() {
  setStatus("reconstruindo…");
  try {
    const scene = await api("/api/build", { config: state.config });
    showError(null);
    state.scene = scene;
    await buildScene(scene);
    renderSliders(scene.coordinates);
    const i = scene.info;
    setStatus(`exo ${i.exo_mass.toFixed(2)} kg · ${i.coordinates.length} coords · ${i.couplings.length} acoplamentos`);
    if (i.warnings.length) showError("Avisos:\n" + i.warnings.join("\n"));
  } catch (e) { showError(e); setStatus("erro"); }
}

function input(type, value, onChange, attrs = {}) {
  const el = document.createElement(type === "select" ? "select" : "input");
  if (type !== "select") el.type = type;
  Object.assign(el, attrs);
  if (type === "checkbox") el.checked = !!value; else if (type !== "select") el.value = value;
  el.addEventListener("change", () => onChange(type === "checkbox" ? el.checked
    : type === "number" ? parseFloat(el.value) : el.value));
  return el;
}
const toHex = (c) => "#" + c.map((x) => Math.round(x * 255).toString(16).padStart(2, "0")).join("");
const fromHex = (h) => [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16) / 255);

function field(label, el) {
  const row = document.createElement("div");
  row.className = "field";
  const l = document.createElement("label");
  l.textContent = label;
  l.title = label;
  row.append(l, el);
  return row;
}

function renderValue(obj, key, path) {
  const v = obj[key];
  const set = (nv) => { obj[key] = nv; scheduleRebuild(); };
  if (key === "sides") {
    const box = document.createElement("div");
    box.className = "vec";
    for (const s of ["r", "l"]) {
      const cb = input("checkbox", v.includes(s), (on) => {
        obj[key] = ["r", "l"].filter((x) => (x === s ? on : obj[key].includes(x)));
        scheduleRebuild();
      });
      const lab = document.createElement("label");
      lab.append(cb, ` ${s === "r" ? "direito" : "esquerdo"}`);
      box.append(lab);
    }
    return field("lados", box);
  }
  if (ENUM_KEYS[key] && typeof v === "string") {
    const sel = input("select", v, set);
    for (const o of state.meta.enums[ENUM_KEYS[key]]) sel.add(new Option(o, o, false, o === v));
    return field(key, sel);
  }
  if (typeof v === "boolean") return field(key, input("checkbox", v, set));
  if (typeof v === "number") return field(key, input("number", v, set, { step: "any" }));
  if (typeof v === "string") {
    if (key === "axis") return field(key, input("text", v, (nv) => set(parseAxis(nv))));
    return field(key, input("text", v, set));
  }
  if (Array.isArray(v) && v.every((x) => typeof x === "number")) {
    if (key.startsWith("color")) return field(key, input("color", toHex(v), (h) => set(fromHex(h))));
    const box = document.createElement("div");
    box.className = "vec";
    v.forEach((x, i) => box.append(input("number", x, (nv) => { obj[key][i] = nv; scheduleRebuild(); }, { step: "any" })));
    if (key === "axis") box.append(input("text", "", (nv) => set(parseAxis(nv)), { placeholder: "human", size: 5 }));
    return field(key, box);
  }
  if (Array.isArray(v)) return renderList(obj, key, path);
  if (v && typeof v === "object") {
    if (key === "overrides" || key === "expressions") return renderJson(obj, key);
    return renderObject(v, key, path, ["segments", "joints", "pelvis_unit"].includes(path.split(".")[0]) && path.split(".").length <= 2);
  }
  return field(key, document.createTextNode(String(v)));
}
function parseAxis(text) {
  const t = text.trim();
  if (!t || t === "human") return "human";
  const nums = t.split(/[ ,;]+/).map(Number);
  return nums.length === 3 && nums.every(Number.isFinite) ? nums : "human";
}

function renderObject(obj, title, path, open = false) {
  const d = document.createElement("details");
  d.open = open;
  const s = document.createElement("summary");
  s.textContent = title;
  d.append(s);
  for (const k of Object.keys(obj)) d.append(renderValue(obj, k, path ? `${path}.${k}` : k));
  return d;
}

function renderList(obj, key, path) {
  const d = document.createElement("details");
  d.open = true;
  const s = document.createElement("summary");
  s.textContent = `${key} (${obj[key].length})`;
  d.append(s);
  obj[key].forEach((item, i) => {
    const sub = renderObject(item, item.name || `${key}[${i}]`, `${path}[${i}]`, true);
    const del = document.createElement("button");
    del.className = "small";
    del.textContent = "remover";
    del.onclick = () => { obj[key].splice(i, 1); renderForm(); scheduleRebuild(); };
    sub.append(del);
    if (key === "cuffs") addMissingCuffKeys(item, sub);
    d.append(sub);
  });
  const add = document.createElement("button");
  add.className = "small";
  add.textContent = `+ adicionar ${key === "cuffs" ? "cuff" : "item"}`;
  add.onclick = () => {
    const last = obj[key][obj[key].length - 1];
    const item = last ? JSON.parse(JSON.stringify(last)) : { name: `${path.split(".")[1] || "seg"}_cuff`, at: 0.5, width: 0.07 };
    item.name = `${item.name || "cuff"}_${obj[key].length + 1}`;
    obj[key].push(item);
    renderForm();
    scheduleRebuild();
  };
  d.append(add);
  return d;
}

// Cuffs only list what they override; offer the remaining cuff_defaults keys on demand.
function addMissingCuffKeys(item, container) {
  const missing = Object.keys(state.config.cuff_defaults).filter((k) => !(k in item));
  if (!missing.length) return;
  const sel = document.createElement("select");
  sel.add(new Option("+ sobrescrever parâmetro…", ""));
  for (const k of missing) sel.add(new Option(k, k));
  sel.onchange = () => {
    if (!sel.value) return;
    item[sel.value] = JSON.parse(JSON.stringify(state.config.cuff_defaults[sel.value]));
    renderForm();
  };
  container.append(sel);
}

function renderJson(obj, key) {
  const ta = document.createElement("textarea");
  ta.value = JSON.stringify(obj[key], null, 1);
  ta.onchange = () => {
    try { obj[key] = JSON.parse(ta.value || "{}"); ta.style.borderColor = ""; scheduleRebuild(); }
    catch { ta.style.borderColor = "red"; }
  };
  return field(key, ta);
}

function renderForm() {
  const form = $("form");
  const openPaths = new Set([...form.querySelectorAll("details[open] > summary")].map((s) => s.textContent));
  form.innerHTML = "";
  const c = state.config;
  form.append(field("name", input("text", c.name, (v) => { c.name = v; })));
  form.append(renderValue(c, "sides", "sides"));
  for (const k of ["segments", "joints", "pelvis_unit", "cuff_defaults", "appearance", "human", "overrides"]) {
    form.append(renderValue(c, k, k));
  }
  if (openPaths.size) form.querySelectorAll("details > summary").forEach((s) => {
    if (openPaths.has(s.textContent)) s.parentElement.open = true;
  });
}

// ------------------------------------------------------------------ pose
let poseBusy = false, posePending = null;
async function sendPose(body) {
  if (poseBusy) { posePending = { ...(posePending || {}), ...body, pose: { ...(posePending?.pose || {}), ...(body.pose || {}) } }; return; }
  poseBusy = true;
  try { applyPose(await api("/api/pose", { autofit: $("autofit").checked, ...body })); showError(null); }
  catch (e) { showError(e); }
  poseBusy = false;
  if (posePending) { const p = posePending; posePending = null; sendPose(p); }
}
function renderSliders(coords) {
  const box = $("sliders");
  box.innerHTML = "";
  for (const c of coords) {
    if (!c.rotational) continue;
    const row = document.createElement("div");
    row.className = "slider";
    const lab = document.createElement("label");
    lab.textContent = c.name;
    const val = document.createElement("span");
    val.textContent = c.value.toFixed(0);
    const sl = input("range", c.value, () => {}, { min: Math.max(c.min, -180), max: Math.min(c.max, 180), step: 0.5 });
    sl.addEventListener("input", () => { val.textContent = (+sl.value).toFixed(0); sendPose({ pose: { [c.name]: +sl.value } }); });
    row.append(lab, sl, val);
    box.append(row);
  }
}
$("autofit").onchange = () => sendPose({});
$("resetPose").onclick = async () => { await sendPose({ reset: true }); renderSliders((await api("/api/scene")).coordinates); };

// ------------------------------------------------------------------ motion
$("loadMotion").onclick = async () => {
  setStatus("ajustando movimento…");
  $("loadMotion").disabled = true;
  try {
    state.motion = await api("/api/motion", { path: $("motionPath").value, step: +$("motionStep").value });
    const n = state.motion.frames.length;
    $("timeSlider").max = n - 1;
    $("timeSlider").disabled = $("play").disabled = false;
    const sel = $("plotCoupling");
    sel.innerHTML = "";
    for (const l of state.motion.frames[0].loads) sel.add(new Option(l.name, l.name));
    showFrame(0);
    setStatus(`${n} quadros · energia máx. dos acoplamentos ${Math.max(...state.motion.energy).toFixed(2)} J`);
  } catch (e) { showError(e); setStatus("erro"); }
  $("loadMotion").disabled = false;
};
function showFrame(i) {
  const f = state.motion.frames[i];
  state.frameIdx = i;
  $("timeSlider").value = i;
  $("timeLabel").textContent = `t = ${f.t.toFixed(3)} s`;
  applyPose(f);
  drawPlot();
}
$("timeSlider").oninput = () => showFrame(+$("timeSlider").value);
$("play").onclick = () => {
  state.playing = !state.playing;
  $("play").textContent = state.playing ? "❚❚" : "▶";
  if (state.playing) tick();
};
function tick() {
  if (!state.playing || !state.motion) return;
  const n = state.motion.frames.length;
  showFrame((state.frameIdx + 1) % n);
  const dt = n > 1 ? (state.motion.frames[1].t - state.motion.frames[0].t) * 1000 : 30;
  setTimeout(tick, Math.max(dt, 15));
}
$("plotCoupling").onchange = drawPlot;
$("plotComp").onchange = drawPlot;
function drawPlot() {
  const cv = $("loadPlot"), ctx = cv.getContext("2d");
  const W = cv.width = cv.clientWidth * devicePixelRatio, H = cv.height = cv.clientHeight * devicePixelRatio;
  ctx.clearRect(0, 0, W, H);
  if (!state.motion) return;
  const name = $("plotCoupling").value, comp = $("plotComp").value;
  const fr = state.motion.frames;
  const ys = fr.map((f) => (f.loads.find((l) => l.name === name) || {})[comp] ?? 0);
  const lo = Math.min(0, ...ys), hi = Math.max(1e-6, ...ys);
  const px = (i) => 30 * devicePixelRatio + (i / Math.max(fr.length - 1, 1)) * (W - 40 * devicePixelRatio);
  const py = (y) => H - 15 * devicePixelRatio - ((y - lo) / (hi - lo)) * (H - 30 * devicePixelRatio);
  const css = getComputedStyle(document.documentElement);
  ctx.strokeStyle = css.getPropertyValue("--line"); ctx.beginPath(); ctx.moveTo(px(0), py(0)); ctx.lineTo(px(fr.length - 1), py(0)); ctx.stroke();
  ctx.strokeStyle = css.getPropertyValue("--accent"); ctx.lineWidth = 2 * devicePixelRatio; ctx.beginPath();
  ys.forEach((y, i) => (i ? ctx.lineTo(px(i), py(y)) : ctx.moveTo(px(i), py(y))));
  ctx.stroke();
  ctx.fillStyle = css.getPropertyValue("--text");
  ctx.beginPath(); ctx.arc(px(state.frameIdx), py(ys[state.frameIdx]), 4 * devicePixelRatio, 0, 7); ctx.fill();
  ctx.font = `${11 * devicePixelRatio}px system-ui`;
  ctx.fillText(`${hi.toFixed(1)}`, 2, py(hi) + 10 * devicePixelRatio);
  ctx.fillText(`${lo.toFixed(1)}`, 2, py(lo));
  ctx.fillText(`${name} ${comp}`, px(0) + 4, 14 * devicePixelRatio);
}

// ------------------------------------------------------------------ header actions
$("loadPreset").onclick = async () => {
  try { state.config = await api(`/api/preset?name=${encodeURIComponent($("preset").value)}`); renderForm(); rebuild(); }
  catch (e) { showError(e); }
};
$("save").onclick = async () => {
  try { const r = await api("/api/save", { path: $("savePath").value }); setStatus(`salvo: ${r.written}`); }
  catch (e) { showError(e); }
};
$("export").onclick = async () => {
  setStatus("exportando…");
  try { const r = await api("/api/export", { path: $("exportPath").value }); setStatus(`exportado: ${r.written}`); }
  catch (e) { showError(e); setStatus("erro"); }
};
document.querySelectorAll("#tabs button").forEach((b) => b.onclick = () => {
  document.querySelectorAll("#tabs button, .tab").forEach((x) => x.classList.remove("active"));
  b.classList.add("active");
  $(`tab-${b.dataset.tab}`).classList.add("active");
  if (b.dataset.tab === "motion") drawPlot();
});

// ------------------------------------------------------------------ boot
(async function boot() {
  try {
    state.meta = await api("/api/meta");
    for (const p of state.meta.presets) $("preset").add(new Option(p, p));
    const scene = await api("/api/scene");
    state.config = scene.config;
    $("preset").value = state.config.name;
    renderForm();
    await buildScene(scene);
    renderSliders(scene.coordinates);
    const i = scene.info;
    setStatus(`exo ${i.exo_mass.toFixed(2)} kg · ${i.coordinates.length} coords · ${i.couplings.length} acoplamentos`);
  } catch (e) { showError(e); setStatus("erro"); }
})();
