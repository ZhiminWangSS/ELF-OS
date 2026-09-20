#!/usr/bin/env python3
"""Local ELF-OS Go2 primitives. step is dry-run unless --execute is supplied."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parent


def summarize_result(rc, interrupted, events):
    """Keep process status, robot state and RPC responses in separate fields."""
    return {'returncode': rc, 'interrupted_or_timeout': interrupted,
            'guard_warnings': [e for e in events if e.get('event') == 'guard_warning'],
            'guard_rejections': [e for e in events if e.get('event') == 'guard_rejected'],
            'rpc_failures': [e for e in events if e.get('event') in ('move', 'stop')
                             and e.get('code') != 0]}


def validate_step(vx, yaw_rate, seconds):
    if not all(math.isfinite(v) for v in (vx, yaw_rate, seconds)):
        raise ValueError("step parameters must be finite")
    if not (0 <= vx <= .2 and abs(yaw_rate) <= .35 and 0 < seconds <= 1.25):
        raise ValueError("step limits: vx 0..0.2, |yaw-rate| <=0.35, seconds (0,1.25]")
    if (vx == 0) == (yaw_rate == 0):
        raise ValueError("select forward OR rotation, one nonzero axis")


DISCRETE_ACTIONS = {
    ('forward', 25, 'cm'): (.20, 0.0, 1.25),
    ('forward', 50, 'cm'): (.20, 0.0, 2.50),
    ('forward', 75, 'cm'): (.20, 0.0, 3.75),
    ('turn_left', 15, 'degree'): (0.0, .35, round(math.radians(15) / .35, 3)),
    ('turn_left', 30, 'degree'): (0.0, .35, round(math.radians(30) / .35, 3)),
    ('turn_left', 45, 'degree'): (0.0, .35, round(math.radians(45) / .35, 3)),
    ('turn_right', 15, 'degree'): (0.0, -.35, round(math.radians(15) / .35, 3)),
    ('turn_right', 30, 'degree'): (0.0, -.35, round(math.radians(30) / .35, 3)),
    ('turn_right', 45, 'degree'): (0.0, -.35, round(math.radians(45) / .35, 3)),
    ('stop', 0, 'none'): (0.0, 0.0, 0.0),
}


def discrete_action_spec(action, value, unit):
    try:
        spec = DISCRETE_ACTIONS[(action, int(value), unit)]
    except (KeyError, TypeError, ValueError):
        raise ValueError('unsupported discrete action')
    return spec


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def validate_observation(path, iface, now=None):
    data = json.loads(Path(path).read_text())
    if data.get('schema') != 'elf.observation.v1' or data.get('boot_id') != boot_id() or data.get('interface') != iface:
        raise ValueError("observation schema, boot or interface mismatch")
    age = (time.monotonic() if now is None else now) - data['started_monotonic_s']
    if not math.isfinite(age) or not 0 <= age <= 60:
        raise ValueError("observation expired; observe and inspect a fresh image")
    image = Path(data['image']['path'])
    if hashlib.sha256(image.read_bytes()).hexdigest() != data['image']['sha256']:
        raise ValueError("observation image changed")
    return data


class Backend:
    def __init__(self, sdk_root, build_dir, runs):
        self.sdk_root = sdk_root.resolve()
        self.build_dir = build_dir.resolve()
        self.binary = self.build_dir / 'elf_go2'
        self.library = self.sdk_root / 'thirdparty/lib' / platform.machine()
        self.runs = runs.resolve()

    def env(self):
        if not (self.library / 'libddsc.so.0').is_file():
            raise RuntimeError('missing SDK DDS library: ' + str(self.library))
        return {'HOME': str(Path.home()), 'PATH': '/usr/local/bin:/usr/bin:/bin',
                'LD_LIBRARY_PATH': str(self.library)}

    def build(self):
        subprocess.run(['cmake', '-S', str(ROOT), '-B', str(self.build_dir),
                        '-DUNITREE_SDK2_ROOT=' + str(self.sdk_root)], check=True)
        subprocess.run(['cmake', '--build', str(self.build_dir), '-j2'], check=True)

    def invoke(self, iface, action, *args, observation=None):
        if not self.binary.is_file():
            raise RuntimeError('build ELF-OS helper first: python3 ' + str(ROOT/'go2.py') + ' build')
        cache = (self.build_dir / 'CMakeCache.txt').read_text()
        if 'UNITREE_SDK2_ROOT:PATH=' + str(self.sdk_root) + '\n' not in cache:
            raise RuntimeError('build uses a different SDK; rebuild with --sdk-root')
        self.runs.mkdir(parents=True, exist_ok=True)
        record = self.runs / (time.strftime('%Y%m%dT%H%M%S') + '-' + uuid.uuid4().hex)
        record.mkdir()
        command = [str(self.binary), iface, action, *map(str, args)]
        (record/'request.json').write_text(json.dumps({'argv': command, 'started_unix_s': time.time(),
                                                     'observation': observation}, indent=2))
        # Keep raw evidence even on timeout, exception or cancellation.
        with (record/'stdout.jsonl').open('w') as out, (record/'stderr.log').open('w') as err:
            process = subprocess.Popen(command, env=self.env(), stdout=out, stderr=err, start_new_session=True)
            failed = False
            def cancel(signum, frame):
                raise KeyboardInterrupt
            previous_term = signal.signal(signal.SIGTERM, cancel)
            try:
                rc = process.wait(timeout=12)
            except (subprocess.TimeoutExpired, KeyboardInterrupt):
                failed = True
                process.send_signal(signal.SIGTERM)
                try:
                    rc = process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill(); rc = process.wait()
                if action == 'step':
                    # Independent stop attempt; no recursive step retry.
                    with (record/'emergency_stop.log').open('w') as stop_log:
                        try:
                            subprocess.run([str(self.binary), iface, 'stop'], env=self.env(),
                                           stdout=stop_log, stderr=subprocess.STDOUT, timeout=5)
                        except subprocess.TimeoutExpired:
                            stop_log.write('STOP UNCONFIRMED: RPC process timed out\n')
            finally:
                signal.signal(signal.SIGTERM, previous_term)
        stdout = (record/'stdout.jsonl').read_text()
        events = []
        for line in stdout.splitlines():
            if line.startswith('{'):
                try:
                    event = json.loads(line)
                    if isinstance(event, dict):
                        events.append(event)
                except json.JSONDecodeError:
                    pass  # A terminated helper may leave a partial line; retain raw log.
        summary = summarize_result(rc, failed, events)
        (record/'result.json').write_text(json.dumps(summary, indent=2) + '\n')
        if rc != 0 or failed:
            raise RuntimeError('Go2 action failed: process_exit_code=' + str(rc)
                               + ', interrupted_or_timeout=' + str(failed)
                               + '; inspect ' + str(record) + '\n'
                               + (record/'stderr.log').read_text())
        return events, record

    def observe(self, iface, output):
        from PIL import Image
        started = time.monotonic()
        output = output.resolve()
        _, image_log = self.invoke(iface, 'image', output)
        with Image.open(output) as img:
            img.verify()
        with Image.open(output) as img:
            width, height = img.size
        events, state_log = self.invoke(iface, 'state')
        state = next(e for e in reversed(events) if e['event'] == 'state')
        data = {'schema': 'elf.observation.v1', 'observation_id': uuid.uuid4().hex,
                'interface': iface, 'boot_id': boot_id(), 'started_monotonic_s': started,
                'received_unix_s': time.time(), 'sensor_stamp': None,
                'image': {'path': str(output), 'width': width, 'height': height,
                          'sha256': hashlib.sha256(output.read_bytes()).hexdigest()},
                'state': state, 'depth': None, 'calibration_id': None,
                'logs': [str(image_log), str(state_log)]}
        metadata = output.with_suffix('.observation.json')
        metadata.write_text(json.dumps(data, indent=2)+'\n')
        return {'image': str(output), 'observation': str(metadata), 'state': state}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['build', 'observe', 'state', 'step', 'stop', 'discrete-action'])
    p.add_argument('--iface', default='eth0')
    p.add_argument('--sdk-root', type=Path, default=Path.home()/'unitree_sdk2-2.0.2')
    p.add_argument('--build-dir', type=Path, default=ROOT/'build')
    p.add_argument('--runs', type=Path, default=ROOT/'runs')
    p.add_argument('--output', type=Path, default=None)
    p.add_argument('--vx', type=float, default=.2)
    p.add_argument('--yaw-rate', type=float, default=0)
    p.add_argument('--seconds', type=float, default=1.25)
    p.add_argument('--observation', type=Path)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--action', dest='discrete_name', choices=['forward', 'turn_left', 'turn_right', 'stop'])
    p.add_argument('--value', type=int, default=0)
    p.add_argument('--unit', default='none')
    args = p.parse_args(argv)
    backend = Backend(args.sdk_root, args.build_dir, args.runs)
    try:
        if args.action == 'build':
            backend.build(); return 0
        if args.action == 'step':
            validate_step(args.vx, args.yaw_rate, args.seconds)
            if not args.execute:
                print(json.dumps({'dry_run': True, 'vx': args.vx, 'yaw_rate': args.yaw_rate,
                                  'seconds': args.seconds, 'nominal_forward_m': args.vx*args.seconds}))
                return 0
            if args.observation is None:
                raise ValueError('step --execute requires --observation from a reviewed fresh observe')
            observation = validate_observation(args.observation, args.iface)
            result = backend.invoke(args.iface, 'step', args.vx, args.yaw_rate, args.seconds, '--execute',
                                    observation=observation)
        elif args.action == 'discrete-action':
            if args.discrete_name is None:
                raise ValueError('discrete-action requires --action')
            vx, yaw_rate, seconds = discrete_action_spec(args.discrete_name, args.value, args.unit)
            if not args.execute:
                print(json.dumps({'dry_run': True, 'action': args.discrete_name,
                                  'value': args.value, 'unit': args.unit, 'vx': vx,
                                  'yaw_rate': yaw_rate, 'seconds': seconds,
                                  'nominal_forward_m': vx * seconds}))
                return 0
            if args.discrete_name == 'stop':
                result = backend.invoke(args.iface, 'stop')
            else:
                if args.observation is None:
                    raise ValueError('discrete-action --execute requires --observation')
                observation = validate_observation(args.observation, args.iface)
                events = []
                remaining = seconds
                while remaining > 1e-9:
                    segment = min(1.25, remaining)
                    segment_events, _ = backend.invoke(
                        args.iface, 'step', vx, yaw_rate, segment, '--execute',
                        observation=observation)
                    events.extend(segment_events)
                    remaining -= segment
                result = (events, Path('discrete-action'))
        elif args.action == 'observe':
            output = args.output or ROOT/'captures'/(uuid.uuid4().hex+'.jpg')
            print(json.dumps(backend.observe(args.iface, output), indent=2)); return 0
        else:
            result = backend.invoke(args.iface, args.action)
        print(json.dumps({'events': result[0], 'log_dir': str(result[1])}, indent=2))
        return 0
    except (ValueError, KeyError, OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr); return 1


if __name__ == '__main__':
    sys.exit(main())
