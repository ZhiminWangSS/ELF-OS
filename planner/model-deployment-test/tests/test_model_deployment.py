import json
import threading
import tempfile
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from unittest.mock import patch

from model_deployment_test.actions import is_explicit_stop, parse_action
from model_deployment_test.contracts import SubgoalRequest
from model_deployment_test.sampling import materialize_frames, sample_indices
from model_deployment_test.inference import InferenceClient


class ModelDeploymentTests(unittest.TestCase):
    def test_strict_actions(self):
        self.assertEqual(parse_action("move forward 25 cm").value, 25)
        self.assertEqual(parse_action("turn left 45 degrees").name, "turn_left")
        self.assertEqual(parse_action("move forward 20 cm").name, "stop")
        self.assertEqual(parse_action("turn left 15 degrees and move forward 25 cm").name, "stop")
        self.assertTrue(is_explicit_stop("stop"))
        self.assertFalse(is_explicit_stop("move forward 20 cm"))

    def test_sampling_keeps_latest_and_pads(self):
        self.assertEqual(sample_indices(3, 8)[-1], 7)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = []
            for i in range(3):
                path = root / ("source%d.jpg" % i); path.write_bytes(bytes([i + 1])); paths.append(path)
            outputs = materialize_frames(paths, root / "input")
            self.assertEqual(len(outputs), 8)
            self.assertEqual(outputs[-1].read_bytes(), b"\x03")

    def test_request_validation(self):
        request = SubgoalRequest(instruction="test")
        request.validate()
        with self.assertRaises(ValueError):
            SubgoalRequest(instruction="", max_decisions=0).validate()

    def test_multipart_inference_request_contains_order_and_hashes(self):
        captured = {}
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                captured["content_type"] = self.headers["Content-Type"]
                captured["body"] = self.rfile.read(int(self.headers["Content-Length"]))
                self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
                self.wfile.write(b'{"action":"move forward 25 cm"}')
            def log_message(self, *_args):
                pass
        server = HTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever); thread.daemon = True; thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                first = root / "a.jpg"; second = root / "b.jpg"
                first.write_bytes(b"first"); second.write_bytes(b"second")
                response = InferenceClient("http://127.0.0.1:%d" % server.server_port).navigate("go", [first, second])
                self.assertEqual(response["action"], "move forward 25 cm")
                self.assertIn("multipart/form-data", captured["content_type"])
                self.assertIn(b'frame_000', captured["body"])
                self.assertIn(b'frame_001', captured["body"])
        finally:
            server.shutdown(); server.server_close(); thread.join()


if __name__ == "__main__":
    unittest.main()
