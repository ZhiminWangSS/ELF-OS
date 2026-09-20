import json
import subprocess
import sys
import time
from pathlib import Path

from .actions import is_explicit_stop, parse_action
from .contracts import SubgoalRequest, SubgoalResult
from .inference import InferenceClient
from .sampling import materialize_frames
from .ssh import SshTunnel


ROOT = Path(__file__).resolve().parents[3]
GO2 = ROOT / "control" / "go2.py"


def _observe(run_dir, index):
    image = run_dir / ("observe_%04d.jpg" % index)
    command = [sys.executable, str(GO2), "observe", "--output", str(image)]
    output = subprocess.check_output(command, text=True)
    return json.loads(output), image


def _action(go2_args, execute, observation_path=None):
    command = [sys.executable, str(GO2), "discrete-action"] + list(go2_args)
    if execute:
        command.append("--execute")
        if observation_path is not None:
            command.extend(["--observation", str(observation_path)])
    return subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def execute_subgoal(request, *, ssh_host=None, execute=False, run_dir=Path("runs/model-deployment-test"), endpoint="http://127.0.0.1:18011"):
    request.validate()
    result = SubgoalResult(subgoal_id=request.subgoal_id)
    started = time.monotonic()
    run_dir = Path(run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    raw = run_dir / "raw_frames"
    sampled = run_dir / "model_input"
    history = []
    tunnel = SshTunnel(ssh_host) if ssh_host else None
    client = InferenceClient(endpoint)
    try:
        if tunnel and execute:
            tunnel.start()
        for decision in range(request.max_decisions):
            if time.monotonic() - started >= request.max_seconds:
                result.status = "budget_exhausted"; break
            observation, image = _observe(raw, len(history))
            history.append(image)
            frames = materialize_frames(history, sampled)
            prediction = client.navigate(request.instruction, frames)
            action = parse_action(prediction["action"])
            record = {"decision": decision + 1, "observation": observation, "prediction": prediction,
                      "action": {"name": action.name, "value": action.value, "unit": action.unit, "raw": action.raw}}
            if action.name == "stop":
                result.status = "model_declared_complete" if is_explicit_stop(prediction.get("action", "")) else "invalid_model_output"
                stop_result = _action(["--action", "stop", "--value", "0", "--unit", "none"], execute)
                record["control"] = {"returncode": stop_result.returncode,
                                     "stdout": stop_result.stdout, "stderr": stop_result.stderr}
                if stop_result.returncode != 0:
                    result.status = "safety_stopped"
                result.steps.append(record); break
            distance = action.value / 100.0 if action.name == "forward" else 0.0
            if result.forward_m + distance > request.max_forward_m:
                result.status = "budget_exhausted"; result.steps.append(record); break
            args = ["--action", action.name, "--value", str(action.value), "--unit", action.unit]
            observation_path = image.with_suffix(".observation.json")
            command_result = _action(args, execute, observation_path)
            record["control"] = {"returncode": command_result.returncode, "stdout": command_result.stdout, "stderr": command_result.stderr}
            result.steps.append(record)
            result.decisions += 1
            result.forward_m += distance
            if command_result.returncode != 0:
                result.status = "safety_stopped"; break
        else:
            result.status = "budget_exhausted"
    except Exception as exc:
        result.status = "inference_failed"
        result.error = str(exc)
        if execute:
            subprocess.run([sys.executable, str(GO2), "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        if tunnel:
            tunnel.close()
        result.elapsed_s = time.monotonic() - started
        (run_dir / "result.json").write_text(json.dumps(result.to_dict(), ensure_ascii=False, indent=2) + "\n")
    return result
