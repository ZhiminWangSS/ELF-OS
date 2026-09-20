import argparse
import json
from pathlib import Path

from .contracts import SubgoalRequest
from .runner import execute_subgoal


def main():
    parser = argparse.ArgumentParser(description="ELF-OS NaVILA model deployment test")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run")
    run.add_argument("--instruction", required=True)
    run.add_argument("--subgoal-id", default="subgoal-1")
    run.add_argument("--ssh-host", required=True)
    run.add_argument("--max-decisions", type=int, required=True)
    run.add_argument("--max-forward-m", type=float, required=True)
    run.add_argument("--max-seconds", type=float, required=True)
    run.add_argument("--run-dir", type=Path, default=Path("runs/model-deployment-test"))
    run.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    request = SubgoalRequest(subgoal_id=args.subgoal_id, instruction=args.instruction,
                             max_decisions=args.max_decisions, max_forward_m=args.max_forward_m,
                             max_seconds=args.max_seconds)
    print(json.dumps(execute_subgoal(request, ssh_host=args.ssh_host, execute=args.execute,
                                     run_dir=args.run_dir).to_dict(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
