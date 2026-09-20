import contextlib
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import go2


class ControlTests(unittest.TestCase):
    def test_robot_code_is_not_rpc_or_process_code(self):
        guard = {'event': 'guard_rejected', 'phase': 'during_move',
                 'reason': 'robot_error_code_nonzero', 'robot_error_code': 2, 'mode': 3}
        result = go2.summarize_result(1, False, [
            {'event': 'state', 'error_code': 2}, {'event': 'move', 'code': 0},
            guard, {'event': 'stop', 'code': 0}])
        self.assertEqual(result['returncode'], 1)
        self.assertEqual(result['guard_rejections'], [guard])
        self.assertEqual(result['rpc_failures'], [])
        rpc = {'event': 'move', 'code': 2}
        result = go2.summarize_result(1, False, [rpc])
        self.assertEqual(result['guard_rejections'], [])
        self.assertEqual(result['rpc_failures'], [rpc])

    def test_dry_run_never_touches_sdk(self):
        with patch.object(go2.Backend, 'invoke') as invoke, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(go2.main(['step']), 0)
            invoke.assert_not_called()

    def test_bad_steps_and_missing_observation_never_touch_sdk(self):
        for args in [['step','--vx','nan'], ['step','--vx','-.1'],
                     ['step','--yaw-rate','.1'], ['step','--seconds','2'],
                     ['step','--execute']]:
            with self.subTest(args=args), patch.object(go2.Backend, 'invoke') as invoke, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(go2.main(args), 1)
                invoke.assert_not_called()

    def test_observation_freshness_identity_and_image_integrity(self):
        with tempfile.TemporaryDirectory() as d:
            image=Path(d)/'image.jpg'; image.write_bytes(b'image fixture')
            path=Path(d)/'observation.json'
            data={'schema':'elf.observation.v1','boot_id':go2.boot_id(),
                  'interface':'eth0','started_monotonic_s':100.,
                  'image':{'path':str(image), 'sha256':hashlib.sha256(image.read_bytes()).hexdigest()}}
            path.write_text(json.dumps(data))
            self.assertEqual(go2.validate_observation(path,'eth0',now=110),data)
            for now,iface in [(161,'eth0'),(99,'eth0'),(110,'wlan0')]:
                with self.assertRaises(ValueError): go2.validate_observation(path,iface,now=now)
            image.write_bytes(b'changed')
            with self.assertRaises(ValueError): go2.validate_observation(path,'eth0',now=110)

    def test_stop_does_not_require_observation_or_execute(self):
        with patch.object(go2.Backend,'invoke',return_value=([{'event':'stop','code':0}],Path('/tmp/log'))) as invoke, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(go2.main(['stop']),0)
            invoke.assert_called_once_with('eth0','stop')

    def test_two_forward_and_rotation_limits(self):
        go2.validate_step(.2,0,1.25)
        go2.validate_step(0,-.35,.5)
        with self.assertRaises(ValueError): go2.validate_step(.2,.35,.5)

    def test_discrete_action_dry_run_supports_long_whitelisted_actions(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(go2.main(['discrete-action', '--action', 'forward',
                                       '--value', '75', '--unit', 'cm']), 0)
        self.assertIn('"seconds": 3.75', output.getvalue())


if __name__=='__main__': unittest.main()
