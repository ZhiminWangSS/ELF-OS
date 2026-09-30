#!/usr/bin/env python3
"""Manual Go2 teleoperation console.

Reuses the deployment safety layer: every motion command goes through
elf_model_deployment.scanner._run (balanceStand wait, DDS-abort retry) and
carries a fresh guarded observation; seek reuses the fast guarded view scan.
"""
import json
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "model-deployment"))

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response, StreamingResponse
from pydantic import BaseModel

from elf_model_deployment.scanner import GO2, _observe, _run, capture_views

RUNS = ROOT / "teleop" / "runs"
CAPTURES = RUNS / "captures"
SCANS = RUNS / "scans"
SESSIONS = RUNS / "sessions"

app = FastAPI(title="ELF-OS Go2 teleop", version="1.0.0")

motion_lock = threading.Lock()          # one motion command at a time
cache_lock = threading.Lock()
latest = {"jpeg": None, "state": None, "observation": None, "ts": 0.0, "error": None}
log_lines = []                          # ring buffer for the UI log
session = {"dir": None, "counter": 0, "label": ""}


def note(message):
    stamp = time.strftime("%H:%M:%S")
    log_lines.append("[%s] %s" % (stamp, message))
    del log_lines[:-60]


def manifest(entry):
    if session["dir"]:
        with open(session["dir"] / "manifest.jsonl", "a") as out:
            out.write(json.dumps(entry, ensure_ascii=False) + "\n")


def record_post_action_frame(kind, label, result_rc):
    """After a macro action, archive the robot's first-person view of the
    new vantage point into the active session (image + observation JSON)."""
    if not session["dir"]:
        return None
    index = session["counter"]
    session["counter"] += 1
    sub = session["dir"] / ("%03d-%s" % (index, kind))
    observation, image = _observe(sub, 0)
    entry = {"ts": time.time(), "kind": kind, "label": label, "returncode": result_rc,
             "frame": str(image), "observation": str(observation["observation"]),
             "state": observation["state"]}
    manifest(entry)
    return str(image)


def camera_loop():
    counter = 0
    while True:
        try:
            observation, image = _observe(CAPTURES, counter)
            counter += 1
            with cache_lock:
                latest["jpeg"] = Path(image).read_bytes()
                latest["state"] = observation["state"]
                latest["observation"] = observation["observation"]
                latest["ts"] = time.monotonic()
                latest["error"] = None
        except Exception as exc:
            with cache_lock:
                latest["error"] = str(exc)
        time.sleep(0.8)


def fresh_observation(max_age_s=30.0):
    with cache_lock:
        path, ts = latest["observation"], latest["ts"]
    if path and time.monotonic() - ts <= max_age_s:
        return path
    observation, _ = _observe(CAPTURES, int(time.time()))
    with cache_lock:
        latest["state"] = observation["state"]
        latest["observation"] = observation["observation"]
        latest["ts"] = time.monotonic()
    return observation["observation"]


class Command(BaseModel):
    action: str            # forward | turn_left | turn_right | stop | rotate
    value: float = 0       # cm / degrees / seconds for rotate


ALLOWED = {
    "forward": [25, 50, 75],
    "turn_left": [15, 30, 45],
    "turn_right": [15, 30, 45],
}


def _run_motion(label, argv, observation=None):
    with motion_lock:
        result = _run(argv, True, observation)
    note("%s -> rc=%s %s" % (label, result.returncode,
                             (result.stderr or "").strip().splitlines()[-1] if result.stderr else ""))
    return result


@app.on_event("startup")
def start_camera():
    CAPTURES.mkdir(parents=True, exist_ok=True)
    SCANS.mkdir(parents=True, exist_ok=True)
    threading.Thread(target=camera_loop, daemon=True).start()


@app.get("/")
def index():
    return FileResponse(Path(__file__).parent / "static" / "index.html",
                        headers={"Cache-Control": "no-store"})


@app.get("/api/stream")
def stream():
    def frames():
        while True:
            with cache_lock:
                jpeg = latest["jpeg"]
            if jpeg:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
            time.sleep(0.8)
    return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/api/state")
def state():
    with cache_lock:
        snap = {k: latest[k] for k in ("state", "ts", "error")}
    return {"state": snap["state"], "age_s": round(time.monotonic() - snap["ts"], 1) if snap["ts"] else None,
            "camera_error": snap["error"], "log": list(log_lines[-30:]),
            "session": {"active": bool(session["dir"]), "label": session["label"],
                        "actions": session["counter"], "dir": str(session["dir"] or "")}}


@app.post("/api/command")
def command(cmd: Command):
    if cmd.action == "stop":
        result = _run_motion("stop", ["--action", "stop", "--value", "0", "--unit", "none"])
    elif cmd.action == "rotate":
        rate = 0.35 if cmd.value > 0 else -0.35
        seconds = min(8.0, round(abs(cmd.value) / 0.35 + 0.15, 3))
        result = _run_motion("rotate %+.0f deg" % cmd.value,
                             ["--yaw-rate", str(rate), "--seconds", str(seconds)],
                             observation=fresh_observation())
    elif cmd.action in ALLOWED:
        if cmd.value not in ALLOWED[cmd.action]:
            raise HTTPException(400, "unsupported %s value %s" % (cmd.action, cmd.value))
        unit = "cm" if cmd.action == "forward" else "degree"
        result = _run_motion("%s %s" % (cmd.action, cmd.value),
                             ["--action", cmd.action, "--value", str(int(cmd.value)), "--unit", unit],
                             observation=fresh_observation())
    else:
        raise HTTPException(400, "unknown action")
    if result.returncode:
        raise HTTPException(409, result.stderr.strip()[-300:])
    frame = record_post_action_frame("action", "%s %s" % (cmd.action, cmd.value), result.returncode)
    return {"ok": result.returncode == 0, "recorded_frame": frame}


@app.post("/api/session/start")
def session_start(body: dict = None):
    if session["dir"]:
        raise HTTPException(409, "session already running: " + str(session["dir"]))
    label = (body or {}).get("label") or "task"
    directory = SESSIONS / (time.strftime("%Y%m%d-%H%M%S") + "-" + label)
    directory.mkdir(parents=True, exist_ok=False)
    session.update(dir=directory, counter=0, label=label)
    note("recording started: " + str(directory))
    manifest({"ts": time.time(), "kind": "session-start", "label": label})
    return {"session": str(directory)}


@app.post("/api/session/stop")
def session_stop():
    if not session["dir"]:
        raise HTTPException(409, "no session running")
    ended = session["dir"]
    manifest({"ts": time.time(), "kind": "session-stop", "actions": session["counter"]})
    session.update(dir=None, counter=0, label="")
    note("recording stopped: " + str(ended))
    return {"session": str(ended), "actions": None}


@app.get("/api/session/status")
def session_status():
    return {"active": bool(session["dir"]), "dir": str(session["dir"] or ""),
            "label": session["label"], "actions": session["counter"]}


@app.post("/api/seek")
def seek():
    if session["dir"]:
        directory = session["dir"] / ("%03d-seek" % session["counter"])
        session["counter"] += 1
    else:
        directory = SCANS / time.strftime("%Y%m%d-%H%M%S")
    try:
        with motion_lock:
            views, record = capture_views(directory, True)
    except Exception as exc:
        note("seek failed: %s" % str(exc)[-160:])
        raise HTTPException(409, str(exc)[-300:])
    note("seek ok, heading_err=%.3f rad" % record.get("heading_error_rad", -1))
    manifest({"ts": time.time(), "kind": "seek", "views": {k: str(v) for k, v in views.items()},
              "heading_error_rad": record.get("heading_error_rad"),
              "turns": len(record.get("turns", []))})
    return {"views": {name: "/api/scan-file?path=%s" % str(path) for name, path in views.items()},
            "heading_error_rad": record.get("heading_error_rad")}


@app.get("/api/scan-file")
def scan_file(path: str):
    resolved = Path(path).resolve()
    if not str(resolved).startswith(str(RUNS.resolve())) or not resolved.is_file():
        raise HTTPException(404, "no such scan file")
    return Response(resolved.read_bytes(), media_type="image/jpeg")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)
