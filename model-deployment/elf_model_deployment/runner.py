import json
import subprocess
import sys
import time
from pathlib import Path

from .actions import action_to_go2_args, parse_seekvln_output
from .contracts import SubgoalRequest, SubgoalResult
from .registry import get_backend
from .sampling import materialize_seekvln_frames
from .scanner import GO2, _observe, _run, capture_views
from .ssh import SshTunnel
from .transport import SeekVLNClient


def execute_subgoal(request, *, ssh_host=None, execute=False, run_dir=Path("runs/model-deployment"), endpoint="http://127.0.0.1:18012"):
    request.validate(); backend = get_backend(request.model_id)
    result = SubgoalResult(subgoal_id=request.subgoal_id, model_id=request.model_id, artifacts=str(Path(run_dir).resolve()))
    started = time.monotonic(); run_dir = Path(run_dir).resolve(); run_dir.mkdir(parents=True, exist_ok=True)
    raw = run_dir / "raw_frames"; sampled = run_dir / "model_input"; raw.mkdir(exist_ok=True)
    history = []; tunnel = SshTunnel(ssh_host) if ssh_host and execute else None
    client = SeekVLNClient(endpoint)
    try:
        if tunnel: tunnel.start()
        for decision in range(request.max_decisions):
            if time.monotonic() - started >= request.max_seconds:
                result.status = "budget_exhausted"; break
            observation, image = _observe(raw, len(history))
            history.append(image)
            frames = materialize_seekvln_frames(history, sampled)
            mode_result = client.navigate("mode", request.instruction, frames)
            mode = mode_result.get("mode")
            views, scan_log = ({}, {})
            if mode == "seek":
                views, scan_log = capture_views(run_dir / "scan" / ("decision-%03d" % decision), execute)
                if not all(Path(path).is_file() for path in views.values()):
                    result.status, result.error_code = "safety_stopped", "seek_views_unavailable"
                    result.steps.append({"decision": decision + 1, "mode": mode,
                                         "mode_result": mode_result, "scan": scan_log})
                    break
            elif mode != "nav":
                raise RuntimeError("invalid_mode_response")
            prediction = client.navigate("action", request.instruction, frames, mode=mode,
                                         views=list(views.values()) if views else None)
            action = parse_seekvln_output(prediction.get("raw_text", prediction.get("action", "")), mode)
            record = {"decision": decision + 1, "observation": observation, "mode": mode,
                      "mode_result": mode_result, "prediction": prediction,
                      "action": action.__dict__, "scan": scan_log}
            if action.name == "invalid":
                result.status, result.error_code = "invalid_model_output", "invalid_model_output"
                result.steps.append(record); break
            if action.is_stop:
                result.status = "model_declared_complete"
                stop = _run(["--action", "stop", "--value", "0", "--unit", "none"], execute)
                record["control"] = {"returncode": stop.returncode, "stdout": stop.stdout, "stderr": stop.stderr}
                result.steps.append(record); break
            distance = action.value / 100.0 if action.name == "forward" else 0.0
            if result.forward_m + distance > request.max_forward_m:
                result.status = "budget_exhausted"; result.steps.append(record); break
            control = _run(action_to_go2_args(action), execute, observation["observation"])
            record["control"] = {"returncode": control.returncode, "stdout": control.stdout, "stderr": control.stderr}
            result.steps.append(record); result.decisions += 1; result.forward_m += distance
            if control.returncode:
                result.status, result.error_code = "safety_stopped", "control_failed"; break
        else:
            result.status = "budget_exhausted"
    except Exception as exc:
        result.status, result.error, result.error_code = "inference_failed", str(exc), "execution_error"
        if execute:
            subprocess.run([sys.executable, str(GO2), "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        if tunnel: tunnel.close()
        result.elapsed_s = time.monotonic() - started
        (run_dir / "result.json").write_text(json.dumps(result.to_dict(), ensure_ascii=False, indent=2) + "\n")
    return result
