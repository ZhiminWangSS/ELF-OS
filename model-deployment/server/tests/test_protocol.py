import hashlib
import io
import json
import unittest

from PIL import Image

from protocol import parse_metadata, validate_request


class ProtocolTests(unittest.TestCase):
    def test_manifest_and_hash(self):
        output = io.BytesIO(); Image.new("RGB", (4, 4), "black").save(output, format="JPEG")
        data = output.getvalue()
        metadata = {"schema": "elf.seekvln-request.v1", "request_id": "r", "phase": "mode",
                    "instruction": "go", "num_history_frames": 1, "num_aux_views": 0,
                    "frames": [{"name": "frame_000", "sha256": hashlib.sha256(data).hexdigest()}]}
        self.assertEqual(validate_request(metadata, {"frame_000": data}), ["frame_000"])
        self.assertEqual(parse_metadata(json.dumps(metadata)), metadata)


if __name__ == "__main__":
    unittest.main()
