"""
DevUI — minimal HTTP dev UI for live pattern testing.

Serves a single-page web UI that lets you launch named movement patterns
with adjustable parameters while the robot is running.  Uses only stdlib
(http.server + threading); no extra pip dependencies.

Usage::

    petctl run --backend robot --dev-ui
    # open http://localhost:8765 in a browser
"""

from __future__ import annotations

import asyncio
import inspect
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from petctl.controller import Controller

# Patterns exposed in the UI (observer-only schemes excluded).
_EXPOSED = [
    "ripple", "pulse", "breathe", "sway", "cascade", "slalom", "coil",
    "twitch", "curl", "spin7", "wander", "explore", "drift",
    "stroke", "stroke-curl", "stroke-ripple", "freeze", "pose",
]

# Maps pattern name → param group that determines which sliders to show.
_PARAM_GROUP: dict[str, str] = {
    "ripple": "osc", "pulse": "osc", "breathe": "osc",
    "sway": "osc", "cascade": "osc", "slalom": "osc", "coil": "osc",
    "twitch": "amp",
    "curl": "curl",
    "spin7": "speed", "wander": "speed", "explore": "speed",
}  # anything not listed → "none"

_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>petctl dev UI</title>
<style>
  body { font-family: monospace; background: #111; color: #ddd; padding: 20px; max-width: 640px; }
  h2 { color: #aef; margin-bottom: 4px; }
  #status { font-size: 0.85em; color: #888; margin-bottom: 16px; }
  #active-label { color: #afa; }
  .pattern-grid { display: flex; flex-wrap: wrap; gap: 6px; margin-bottom: 18px; }
  button { background: #222; color: #ccc; border: 1px solid #444; padding: 6px 12px;
           cursor: pointer; font-family: monospace; font-size: 0.9em; border-radius: 3px; }
  button:hover { background: #333; }
  button.selected { background: #1a3a1a; border-color: #4a8; color: #afa; }
  button.action { background: #1a1a3a; border-color: #48a; color: #adf; }
  button.action:hover { background: #222a44; }
  button.stop { background: #3a1a1a; border-color: #a44; color: #faa; }
  button.stop:hover { background: #442222; }
  #params { margin-bottom: 18px; min-height: 60px; }
  .param-row { margin: 8px 0; display: flex; align-items: center; gap: 10px; }
  .param-row label { width: 140px; color: #aaa; }
  .param-row input[type=range] { flex: 1; accent-color: #4a8; }
  .param-row .val { width: 50px; text-align: right; color: #efa; }
  #launch-row { display: flex; gap: 10px; margin-top: 6px; }
</style>
</head>
<body>
<h2>petctl dev UI</h2>
<div id="status">active: <span id="active-label">—</span></div>

<div class="pattern-grid" id="pattern-grid"></div>

<div id="params"></div>

<div id="launch-row">
  <button class="action" onclick="launch()">&#9654; Launch</button>
  <button class="stop" onclick="stop()">&#9632; Stop</button>
</div>

<script>
const PATTERNS = """ + json.dumps(_EXPOSED) + """;
const GROUPS   = """ + json.dumps(_PARAM_GROUP) + """;

const PARAM_DEFS = {
  osc:   [
    { id: "amplitude_deg", label: "amplitude_deg", min: 0,    max: 90,  step: 1,    def: 40,  unit: "°" },
    { id: "hz",            label: "hz",            min: 0.05, max: 2.0, step: 0.05, def: 0.4, unit: "Hz" },
  ],
  amp:   [
    { id: "amplitude_deg", label: "amplitude_deg", min: 0, max: 90, step: 1, def: 30, unit: "°" },
  ],
  speed: [
    { id: "speed_deg_per_s", label: "speed_deg_per_s", min: 5, max: 180, step: 5, def: 45, unit: "°/s" },
  ],
  curl:  [
    { id: "target_deg", label: "target_deg", min: -90, max: 90, step: 1,   def: 70,  unit: "°" },
    { id: "ramp_s",     label: "ramp_s",     min: 0.5, max: 20, step: 0.5, def: 8.0, unit: "s" },
  ],
  none:  [],
};

let selected = null;

// Build pattern buttons
const grid = document.getElementById("pattern-grid");
PATTERNS.forEach(name => {
  const btn = document.createElement("button");
  btn.id = "btn-" + name;
  btn.textContent = name;
  btn.onclick = () => selectPattern(name);
  grid.appendChild(btn);
});

function selectPattern(name) {
  selected = name;
  document.querySelectorAll(".pattern-grid button").forEach(b => b.classList.remove("selected"));
  const btn = document.getElementById("btn-" + name);
  if (btn) btn.classList.add("selected");
  renderParams(name);
}

function renderParams(name) {
  const group = GROUPS[name] || "none";
  const defs  = PARAM_DEFS[group] || [];
  const el    = document.getElementById("params");
  el.innerHTML = "";
  defs.forEach(p => {
    const row = document.createElement("div");
    row.className = "param-row";
    const val = document.createElement("span");
    val.className = "val";
    val.id = "val-" + p.id;
    val.textContent = p.def + p.unit;
    const slider = document.createElement("input");
    slider.type  = "range";
    slider.id    = "slider-" + p.id;
    slider.min   = p.min;
    slider.max   = p.max;
    slider.step  = p.step;
    slider.value = p.def;
    slider.oninput = () => { val.textContent = slider.value + p.unit; };
    const lbl = document.createElement("label");
    lbl.textContent = p.label;
    row.appendChild(lbl);
    row.appendChild(slider);
    row.appendChild(val);
    el.appendChild(row);
  });
}

function gatherParams() {
  const params = {};
  document.querySelectorAll("#params input[type=range]").forEach(s => {
    const key = s.id.replace("slider-", "");
    params[key] = parseFloat(s.value);
  });
  return params;
}

function launchPattern(name, extra) {
  const body = Object.assign({ pattern: name }, extra || {});
  fetch("/launch", { method: "POST",
                     headers: { "Content-Type": "application/json" },
                     body: JSON.stringify(body) })
    .then(r => r.json())
    .then(d => { document.getElementById("active-label").textContent = d.scheme || "?"; })
    .catch(console.error);
}

function launch() {
  if (!selected) return;
  launchPattern(selected, gatherParams());
}

function stop() {
  fetch("/stop", { method: "POST" })
    .then(r => r.json())
    .then(d => { document.getElementById("active-label").textContent = d.scheme || "?"; })
    .catch(console.error);
}

// Poll active scheme every 2 s
function pollStatus() {
  fetch("/status").then(r => r.json())
    .then(d => {
      document.getElementById("active-label").textContent = d.scheme || "?";
      const active = d.scheme;
      document.querySelectorAll(".pattern-grid button").forEach(b => {
        const name = b.id.replace("btn-", "");
        b.classList.toggle("selected", name === active && name === selected);
      });
    }).catch(() => {});
}
setInterval(pollStatus, 2000);
pollStatus();
</script>
</body>
</html>
"""


_CONTROL_MOVEMENTS = sorted([
    "freeze", "home", "breathe", "pulse", "ripple", "sway", "cascade",
    "slalom", "twitch", "coil", "curl_right", "curl_left", "wander", "drift",
    "stroke", "stroke-curl", "stroke-ripple", "yield-stiff", "pose",
])

_CONTROL_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>petctl control</title>
<style>
  body { font-family: monospace; background: #111; color: #ddd; padding: 20px; max-width: 640px; }
  h2 { color: #aef; margin-bottom: 4px; }
  #status { font-size: 0.85em; color: #888; margin-bottom: 16px; }
  #active-label { color: #afa; }
  .pattern-grid { display: flex; flex-wrap: wrap; gap: 6px; margin-bottom: 18px; }
  button { background: #222; color: #ccc; border: 1px solid #444; padding: 6px 12px;
           cursor: pointer; font-family: monospace; font-size: 0.9em; border-radius: 3px; }
  button:hover { background: #333; }
  button.selected { background: #1a3a1a; border-color: #4a8; color: #afa; }
  button.action { background: #1a1a3a; border-color: #48a; color: #adf; }
  button.action:hover { background: #222a44; }
  button.stop { background: #3a1a1a; border-color: #a44; color: #faa; }
  button.stop:hover { background: #442222; }
  .param-row { margin: 10px 0; display: flex; align-items: center; gap: 10px; }
  .param-row label { width: 80px; color: #aaa; }
  .param-row input[type=range] { flex: 1; accent-color: #4a8; }
  .param-row .val { width: 40px; text-align: right; color: #efa; }
  #launch-row { display: flex; gap: 10px; margin-top: 14px; }
</style>
</head>
<body>
<h2>petctl control</h2>
<div id="status">active: <span id="active-label">—</span></div>
<div class="pattern-grid" id="pattern-grid"></div>
<div class="param-row">
  <label>intensity</label>
  <input type="range" id="intensity" min="0" max="1" step="0.05" value="0.5"
         oninput="document.getElementById('ival').textContent=parseFloat(this.value).toFixed(2)">
  <span class="val" id="ival">0.50</span>
</div>
<div class="param-row">
  <label>speed</label>
  <input type="range" id="speed" min="0.05" max="1" step="0.05" value="0.4"
         oninput="document.getElementById('sval').textContent=parseFloat(this.value).toFixed(2)">
  <span class="val" id="sval">0.40</span>
</div>
<div id="launch-row">
  <button class="action" onclick="launch()">&#9654; Launch</button>
  <button class="stop" onclick="stop()">&#9632; Stop</button>
</div>
<script>
const MOVEMENTS = """ + json.dumps(_CONTROL_MOVEMENTS) + """;
let selected = null;
const grid = document.getElementById("pattern-grid");
MOVEMENTS.forEach(function(name) {
  const btn = document.createElement("button");
  btn.id = "btn-" + name;
  btn.textContent = name;
  btn.onclick = function() {
    selected = name;
    document.querySelectorAll(".pattern-grid button").forEach(function(b) { b.classList.remove("selected"); });
    btn.classList.add("selected");
  };
  grid.appendChild(btn);
});
function launch() {
  if (!selected) return;
  fetch("/command", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({
      movement: selected,
      intensity: parseFloat(document.getElementById("intensity").value),
      speed: parseFloat(document.getElementById("speed").value)
    })
  }).then(function(r) { return r.json(); })
    .then(function(d) { document.getElementById("active-label").textContent = d.scheme || d.error || "?"; })
    .catch(console.error);
}
function stop() {
  fetch("/stop", {method: "POST"})
    .then(function(r) { return r.json(); })
    .then(function(d) { document.getElementById("active-label").textContent = d.scheme || "?"; })
    .catch(console.error);
}
function pollStatus() {
  fetch("/status").then(function(r) { return r.json(); })
    .then(function(d) { document.getElementById("active-label").textContent = d.scheme || "?"; })
    .catch(function() {});
}
setInterval(pollStatus, 2000);
pollStatus();
</script>
</body>
</html>
"""


class DevUI:
    """Minimal HTTP dev UI for live pattern testing."""

    def __init__(self, controller: "Controller", port: int = 8765) -> None:
        self._controller = controller
        self._port = port
        self._server: HTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        ctrl = self._controller

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/":
                    body = _HTML.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/control":
                    body = _CONTROL_HTML.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/status":
                    body = json.dumps({"scheme": ctrl.scheme.name}).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_response(404)
                    self.end_headers()

            def do_POST(self):
                if self.path == "/launch":
                    length = int(self.headers.get("Content-Length", 0))
                    data = json.loads(self.rfile.read(length))
                    name = data.get("pattern", "")
                    response = _launch(ctrl, name, data)
                    body = json.dumps(response).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/stop":
                    response = _stop(ctrl)
                    body = json.dumps(response).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/command":
                    length = int(self.headers.get("Content-Length", 0))
                    data = json.loads(self.rfile.read(length))
                    response = _command(ctrl, data)
                    body = json.dumps(response).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, *args):
                pass  # suppress access logs

        self._server = HTTPServer(("", self._port), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True, name="dev-ui"
        )
        self._thread.start()

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()


def _stop(controller: "Controller") -> dict:
    """Disable all motor torques and deactivate the TX loop for every motor."""
    loop = getattr(controller, "_event_loop", None)
    if loop is not None:
        try:
            asyncio.run_coroutine_threadsafe(
                controller.backend.disable_torques(), loop
            ).result(timeout=2.0)
        except Exception:
            pass
    return {"scheme": "stopped"}


def _command(controller: "Controller", data: dict) -> dict:
    """Hot-swap to a pattern using normalized intensity/speed params (Ollama vocabulary)."""
    from petctl.schemes.ollama_scheme import _make_pattern, _VALID_MOVEMENTS

    motion = str(data.get("movement", "")).strip().lower()
    if motion not in _VALID_MOVEMENTS:
        return {"error": f"unknown movement: {motion}"}

    intensity = max(0.0, min(1.0, float(data.get("intensity", 0.5))))
    speed = max(0.05, min(1.0, float(data.get("speed", 0.4))))
    pattern = _make_pattern(motion, intensity, speed)
    controller.set_scheme(pattern)
    return {"scheme": pattern.name}


def _launch(controller: "Controller", name: str, params: dict) -> dict:
    """Instantiate the named pattern with filtered params and hot-swap it in.

    Re-enables motors first in case they were previously stopped.
    """
    from petctl.schemes.patterns import ALL_PATTERNS

    cls = next((c for c in ALL_PATTERNS if c.name == name), None)
    if cls is None:
        return {"error": f"unknown pattern: {name}"}

    # Re-enable any motors that were disabled by stop.
    backend = controller.backend
    loop = getattr(controller, "_event_loop", None)
    if loop is not None and hasattr(backend, "enable_motor"):
        ids = getattr(backend, "_discovered_motors", None) or list(range(1, 8))
        for mid in ids:
            try:
                asyncio.run_coroutine_threadsafe(
                    backend.enable_motor(mid), loop
                ).result(timeout=1.0)
            except Exception:
                pass

    valid = inspect.signature(cls).parameters
    filtered = {k: v for k, v in params.items() if k in valid and k != "self"}
    try:
        controller.set_scheme(cls(**filtered))
    except Exception as e:
        return {"error": str(e)}
    return {"scheme": controller.scheme.name}
