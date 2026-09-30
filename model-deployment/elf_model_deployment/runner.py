import json
import subprocess
import sys
import time
from pathlib import Path

from .actions import action_to_go2_args, parse_seekvln_prediction
from .contracts import SubgoalRequest, SubgoalResult
from .registry import get_backend
from .sampling import materialize_seekvln_frames, prepare_model_frame
from .scanner import GO2, _observe, _run, capture_views
from .ssh import SshTunnel
from .transport import SeekVLNClient


def _execute_navila(request, ssh_host, execute, run_dir, endpoint):
    """Use the preserved NaVILA adapter while exposing the common contract."""
    legacy_root = Path(__file__).resolve().parents[2] / "planner" / "model-deployment-test"
    if str(legacy_root) not in sys.path:
        sys.path.insert(0, str(legacy_root))
    from model_deployment_test.contracts import SubgoalRequest as LegacyRequest
    from model_deployment_test.runner import execute_subgoal as legacy_execute
    legacy_request = LegacyRequest(schema=request.schema, subgoal_id=request.subgoal_id,
                                   instruction=request.instruction, max_decisions=request.max_decisions,
                                   max_forward_m=request.max_forward_m, max_seconds=request.max_seconds)
    legacy = legacy_execute(legacy_request, ssh_host=ssh_host, execute=execute, run_dir=run_dir,
                            endpoint=endpoint or "http://127.0.0.1:18011")
    return SubgoalResult(subgoal_id=request.subgoal_id, model_id="navila", status=legacy.status,
                         decisions=legacy.decisions, forward_m=legacy.forward_m,
                         elapsed_s=legacy.elapsed_s, steps=legacy.steps, error=legacy.error,
                         artifacts=str(Path(run_dir).resolve()))


def execute_subgoal(request, *, ssh_host=None, execute=False, run_dir=Path("runs/model-deployment"), endpoint=None):
    request.validate(); backend = get_backend(request.model_id)
    if backend.model_id == "navila":
        return _execute_navila(request, ssh_host, execute, run_dir, endpoint)
    result = SubgoalResult(subgoal_id=request.subgoal_id, model_id=request.model_id, artifacts=str(Path(run_dir).resolve()))
    started = time.monotonic(); run_dir = Path(run_dir).resolve(); run_dir.mkdir(parents=True, exist_ok=True)
    raw = run_dir / "raw_frames"; sampled = run_dir / "model_input"; raw.mkdir(exist_ok=True)
    history = []; tunnel = SshTunnel(ssh_host) if ssh_host else None
    client = SeekVLNClient(endpoint or "http://127.0.0.1:18012")
    try:
        if tunnel: tunnel.start()
        for decision in range(request.max_decisions):
            if time.monotonic() - started >= request.max_seconds:
                result.status = "budget_exhausted"; break
            observation, image = _observe(raw, len(history))
            history.append(image)
            frames = materialize_seekvln_frames(history, sampled / ("decision-%03d" % decision))
            mode_result = client.navigate("mode", request.instruction, frames)
            mode = mode_result.get("mode")
            views, scan_log = ({}, {})
            if mode == "seek":
                try:
                    views, scan_log = capture_views(run_dir / "scan" / ("decision-%03d" % decision), execute,
                                                    alive=(lambda: tunnel.alive) if tunnel else None)
                except Exception as exc:
                    if execute:
                        subprocess.run([sys.executable, str(GO2), "stop"], stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL)
                    result.status, result.error, result.error_code = "safety_stopped", str(exc), "view_scan_failed"
                    result.steps.append({"decision": decision + 1, "mode": mode,
                                         "mode_result": mode_result, "scan": {"error": str(exc)}})
                    break
                if not all(Path(path).is_file() for path in views.values()):
                    result.status, result.error_code = "safety_stopped", "seek_views_unavailable"
                    result.steps.append({"decision": decision + 1, "mode": mode,
                                         "mode_result": mode_result, "scan": scan_log})
                    break
            elif mode != "nav":
                raise RuntimeError("invalid_mode_response")
            control_observation = observation
            if mode == "seek" and execute:
                # The guarded scan outlives the Go2 executor's 60 s observation
                # freshness window; re-observe at the restored heading so the
                # control step passes the guard, and let that forward view join
                # the history used by later decisions.
                control_observation, post_scan_image = _observe(raw, len(history))
                history.append(post_scan_image)
            model_views = None
            if views:
                view_dir = sampled / ("decision-%03d-views" % decision)
                model_views = [prepare_model_frame(Path(path), view_dir / ("view_%03d.jpg" % index))
                               for index, path in enumerate(views.values())]
            prediction = client.navigate("action", request.instruction, frames, mode=mode, views=model_views)
            action = parse_seekvln_prediction(prediction, mode)
            record = {"decision": decision + 1, "observation": observation, "mode": mode,
                      "mode_result": mode_result, "prediction": prediction,
                      "action": action.__dict__, "scan": scan_log}
            if control_observation is not observation:
                record["post_scan_observation"] = control_observation
            if action.name == "invalid":
                result.status, result.error_code = "invalid_model_output", "invalid_model_output"
                stop = _run(["--action", "stop", "--value", "0", "--unit", "none"], execute)
                record["control"] = {"returncode": stop.returncode, "stdout": stop.stdout, "stderr": stop.stderr}
                result.steps.append(record); break
            if action.is_stop:
                result.status = "model_declared_complete"
                stop = _run(["--action", "stop", "--value", "0", "--unit", "none"], execute)
                record["control"] = {"returncode": stop.returncode, "stdout": stop.stdout, "stderr": stop.stderr}
                result.steps.append(record); break
            distance = action.value / 100.0 if action.name == "forward" else 0.0
            if result.forward_m + distance > request.max_forward_m:
                result.status = "budget_exhausted"; result.steps.append(record); break
            control = _run(action_to_go2_args(action), execute, control_observation["observation"],
                           alive=(lambda: tunnel.alive) if tunnel else None)
            record["control"] = {"returncode": control.returncode, "stdout": control.stdout, "stderr": control.stderr}
            if control.returncode:
                result.steps.append(record)
                result.status, result.error_code = "safety_stopped", "control_failed"; break
            if tunnel and not tunnel.alive:
                if execute:
                    subprocess.run([sys.executable, str(GO2), "stop"], stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
                record["tunnel_alive_after_action"] = False
                result.steps.append(record)
                result.status, result.error_code = "safety_stopped", "ssh_tunnel_lost"; break
            post_observation, post_image = _observe(raw, len(history))
            history.append(post_image)
            record["post_observation"] = post_observation
            result.steps.append(record); result.decisions += 1; result.forward_m += distance
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
