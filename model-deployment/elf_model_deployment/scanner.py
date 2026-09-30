import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


GO2 = Path(__file__).resolve().parents[2] / "control" / "go2.py"


def _wait_until_walkable(timeout_s=45.0, settle_s=0.5, max_speed=0.05):
    """Wait out the stance-adjustment period after motion or remote control:
    the sport mode flaps between balanceStand (1) and locomotion (3) with
    small residual velocities while the robot repositions its feet."""
    def settled(events):
        try:
            state = next(e for e in reversed(events) if e.get("event") == "state")
            speed = sum(v * v for v in state.get("velocity", [])) ** 0.5
            return state.get("mode") == 1 and state.get("error_code") == 0 and speed <= max_speed
        except (StopIteration, AttributeError, KeyError, TypeError):
            return False
    deadline = time.monotonic() + timeout_s
    stable = 0
    while time.monotonic() < deadline and stable < 2:
        output = subprocess.run([sys.executable, str(GO2), "state"], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        try:
            events = json.loads(output.stdout).get("events", [])
        except ValueError:
            events = []
        stable = stable + 1 if settled(events) else 0
        time.sleep(settle_s)
    return stable >= 2


SCAN_YAW_RATE = 0.30  # gentler than the 0.35 limit: sustained in-place spins
                     # at max rate have tripped the sport service into damping


def _run_raw(argv, execute, observation=None, alive=None):
    """Execute one go2.py command with walkable-wait and transient retry."""
    command = [sys.executable, str(GO2)] + list(argv)
    if execute:
        command.append("--execute")
        if observation:
            command.extend(["--observation", str(observation)])
    def once():
        if not execute or alive is None:
            return subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process = subprocess.Popen(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   start_new_session=True)
        while process.poll() is None:
            if not alive():
                os.killpg(process.pid, signal.SIGTERM)
                stdout, stderr = process.communicate()
                subprocess.run([sys.executable, str(GO2), "stop"], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
                return subprocess.CompletedProcess(command, 125, stdout, stderr + "\nSSH tunnel lost")
            time.sleep(.05)
        stdout, stderr = process.communicate()
        return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)
    if execute and argv[1:3] != ["--action", "stop"]:
        # Emergency stop never waits; every other motion waits out the
        # stance-adjustment period first.
        if not _wait_until_walkable():
            raise RuntimeError("robot did not return to balanceStand (mode 1) in time")
        # Let the state poller's DDS entities finish teardown before the
        # motion helper joins the same DDS domain.
        time.sleep(1.5)
    result = once()
    for _ in range(2):
        if result.returncode == 0 or not execute:
            break
        stderr = result.stderr or ""
        dds_abort = "process_exit_code=-6" in stderr
        # In-place spins sporadically trip the sport service into damping
        # (mode 0) mid-motion; it self-recovers, so wait and retry once.
        transient_damping = argv[0] == "rotate" and "mode_not_allowed" in stderr
        if not (dds_abort or transient_damping):
            break
        subprocess.run([sys.executable, str(GO2), "stop"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        if transient_damping and not _wait_until_walkable():
            break
        time.sleep(2.0)
        result = once()
    return result


def _run(args, execute, observation=None, alive=None):
    return _run_raw(["discrete-action"] + list(args), execute, observation=observation, alive=alive)


def _observe(directory, index):
    image = Path(directory) / ("scan_%03d.jpg" % index)
    from PIL import Image
    last_error = None
    for _ in range(3):
        # Retries cover both camera-side failures (truncated or missing JPEG,
        # image RPC timeout) and transient state-reader timeouts.
        try:
            output = subprocess.check_output(
                [sys.executable, str(GO2), "observe", "--output", str(image)], text=True)
            with Image.open(image) as frame:
                frame.load()
            return json.loads(output), image
        except Exception as exc:
            last_error = exc
            time.sleep(.5)
    raise RuntimeError("observe failed after retries: %s (%s)" % (image, last_error))


def _angle_delta(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def capture_views(directory, execute, alive=None):
    """Fast guarded scan: one 90-degree sweep per view, at most one small
    closed-loop correction each, restoring the heading before returning."""
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
    if not execute:
        return {"left": directory / "dry-left.jpg", "forward": directory / "dry-forward.jpg",
                "right": directory / "dry-right.jpg"}, {"dry_run": True}
    initial, forward = _observe(directory, 0)
    records = {"forward": initial, "turns": []}
    next_index = [1]

    def rotate_by(delta_rad, observation):
        """One continuous rotation covering delta_rad, then re-observe."""
        rate = SCAN_YAW_RATE if delta_rad > 0 else -SCAN_YAW_RATE
        # Slight overrun bias counters spin-up lag; the follow-up correction
        # trims any overshoot.
        seconds = min(8.0, round(abs(delta_rad) / SCAN_YAW_RATE + 0.15, 3))
        result = _run_raw(["rotate", "--yaw-rate", str(rate), "--seconds", str(seconds)],
                          True, observation=observation["observation"], alive=alive)
        if result.returncode:
            raise RuntimeError("view scan rotation failed: " + result.stderr.strip())
        time.sleep(1.0)
        state, image = _observe(directory, next_index[0])
        next_index[0] += 1
        records["turns"].append({"action": "rotate", "delta_rad": round(delta_rad, 3), "state": state})
        return state, image

    def sweep(target_yaw, observation):
        """Rotate to target_yaw in one continuous motion, then closed-loop
        trims. Turn execution varies widely across floors (60-110%), so allow
        several proportional trims instead of a single correction."""
        error = _angle_delta(target_yaw, float(observation["state"]["rpy"][2]))
        if abs(error) <= math.radians(10):
            return observation, None
        observation, image = rotate_by(error, observation)
        for _ in range(4):
            error = _angle_delta(target_yaw, float(observation["state"]["rpy"][2]))
            if abs(error) <= math.radians(10):
                break
            observation, image = rotate_by(error, observation)
        if abs(_angle_delta(target_yaw, float(observation["state"]["rpy"][2]))) > math.radians(10):
            raise RuntimeError("view scan could not reach target heading")
        return observation, image

    initial_yaw = float(initial["state"]["rpy"][2])
    try:
        left_state, left = sweep(initial_yaw + math.pi / 2, initial)
        center_state, _ = sweep(initial_yaw, left_state)
        right_state, right = sweep(initial_yaw - math.pi / 2, center_state)
        restore_state, _ = sweep(initial_yaw, right_state)
    except Exception:
        subprocess.run([sys.executable, str(GO2), "stop"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
        raise
    if left is None or right is None:
        raise RuntimeError("view scan did not capture side views")
    records.update(left=left_state, right=right_state, restored=restore_state)
    heading_error = abs(_angle_delta(initial_yaw, float(restore_state["state"]["rpy"][2])))
    records["heading_error_rad"] = heading_error
    if heading_error > .35:
        raise RuntimeError("view scan heading restoration not verified")
    return {"left": left, "forward": forward, "right": right}, records
