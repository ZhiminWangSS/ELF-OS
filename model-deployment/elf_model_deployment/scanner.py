import json
import subprocess
import sys
from pathlib import Path


GO2 = Path(__file__).resolve().parents[2] / "control" / "go2.py"


def _run(args, execute, observation=None):
    command = [sys.executable, str(GO2), "discrete-action"] + list(args)
    if execute:
        command.append("--execute")
        if observation:
            command.extend(["--observation", str(observation)])
    return subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def _observe(directory, index):
    image = Path(directory) / ("scan_%03d.jpg" % index)
    output = subprocess.check_output([sys.executable, str(GO2), "observe", "--output", str(image)], text=True)
    return json.loads(output), image


def capture_views(directory, execute):
    """Capture left/right views with a fixed 90-degree scan and restoration."""
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
    if not execute:
        return {"left": directory / "dry-left.jpg", "forward": directory / "dry-forward.jpg",
                "right": directory / "dry-right.jpg"}, {"dry_run": True}
    initial, forward = _observe(directory, 0)
    records = {"forward": initial}
    def turn(name, value, index, observation):
        args = ["--action", name, "--value", str(value), "--unit", "degree"]
        result = _run(args, True, observation["observation"])
        if result.returncode:
            raise RuntimeError("view scan turn failed: " + result.stderr.strip())
        return _observe(directory, index)
    # Two 45-degree primitives make a 90-degree scan while retaining Go2 limits.
    left_state, left = turn("turn_left", 45, 1, initial)
    left_state, left = turn("turn_left", 45, 2, left_state)
    back_state, _ = turn("turn_right", 45, 3, left_state)
    back_state, _ = turn("turn_right", 45, 4, back_state)
    right_state, right = turn("turn_right", 45, 5, back_state)
    right_state, right = turn("turn_right", 45, 6, right_state)
    restore_state, _ = turn("turn_left", 45, 7, right_state)
    restore_state, _ = turn("turn_left", 45, 8, restore_state)
    records.update(left=left_state, right=right_state, restored=restore_state)
    if abs(float(restore_state["state"]["rpy"][2]) - float(initial["state"]["rpy"][2])) > .35:
        raise RuntimeError("view scan heading restoration not verified")
    return {"left": left, "forward": forward, "right": right}, records
