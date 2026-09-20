#!/usr/bin/env python3
"""Run one Nav2 goal without the short supervised-test session limit."""
import argparse
import math
import os
import platform
import signal
import subprocess
import time
from pathlib import Path

from hardware_test import ROOT, check


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', required=True, type=float)
    parser.add_argument('--y', required=True, type=float)
    parser.add_argument('--yaw-deg', default=0.0, type=float)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (args.x, args.y, args.yaw_deg)):
        raise SystemExit('Goal values must be finite')
    if not args.execute:
        raise SystemExit('--execute is required to send a long-distance goal')

    report = check()
    print(report, flush=True)
    if not report['ready_for_controlled_motion_test']:
        raise SystemExit('Preflight failed; navigation bridge not started')

    worker_env = {
        'HOME': str(Path.home()),
        'PATH': '/usr/bin:/bin',
        'LD_LIBRARY_PATH': str(Path.home() / 'unitree_sdk2-2.0.2/thirdparty/lib' / platform.machine()),
    }
    bridge_env = os.environ.copy()
    bridge_env['PATH'] = '/usr/bin:/bin'
    runtime = Path(f'/tmp/elf-go2-{os.getuid()}')
    socket_path = runtime / 'nav.sock'
    log_dir = ROOT / 'log' / time.strftime('longdistance_%Y%m%d_%H%M%S')
    log_dir.mkdir()
    worker = bridge = goal = None
    try:
        with (log_dir / 'sdk.jsonl').open('w') as sdk_log, (log_dir / 'ros.log').open('w') as ros_log:
            worker = subprocess.Popen([str(ROOT / 'sdk/build/nav_worker'), 'eth0', '--execute'], env=worker_env, stdout=sdk_log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 3
            while not socket_path.exists() and time.monotonic() < deadline:
                if worker.poll() is not None:
                    raise RuntimeError('SDK worker exited; see sdk.jsonl')
                time.sleep(.02)
            if worker.poll() is not None or not socket_path.exists():
                raise RuntimeError('SDK worker socket unavailable')
            bridge = subprocess.Popen([
                '/usr/bin/python3', str(ROOT / 'tools/velocity_bridge.py'), '--socket', str(socket_path),
                '--ros-args', '-r', '__node:=longdistance_velocity_bridge',
                '-r', '/navigation/guarded_cmd_vel:=/navigation/sdk_guarded_cmd_vel',
                '-r', '/navigation/bridge_status:=/navigation/sdk_bridge_status',
            ], env=bridge_env, stdout=ros_log, stderr=subprocess.STDOUT)
            distance = math.hypot(args.x - report['base_xy'][0], args.y - report['base_xy'][1])
            command = [
                '/usr/bin/python3', str(ROOT / 'tools/goal.py'), '--x', str(args.x), '--y', str(args.y),
                '--yaw-deg', str(args.yaw_deg), '--navigate', '--timeout', '0',
                '--max-distance', str(distance + .05), '--allow-long-distance',
            ]
            goal = subprocess.Popen(command, env=bridge_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            output, _ = goal.communicate()
            (log_dir / 'goal.log').write_text(output)
            print(output, end='', flush=True)
            if goal.returncode != 0:
                raise RuntimeError('Nav2 did not complete; see goal.log')
    finally:
        for child in (goal, bridge, worker):
            if child and child.poll() is None:
                child.send_signal(signal.SIGINT)
        for child in (goal, bridge, worker):
            if child:
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=5)
        subprocess.run(['/usr/bin/python3', str(ROOT / 'tools/goal.py'), '--cancel'], env=bridge_env, timeout=8, check=False)
        print(f'Long-distance session closed. Logs: {log_dir}', flush=True)


if __name__ == '__main__':
    main()
