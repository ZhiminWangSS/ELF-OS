import hashlib
import json
import time
import urllib.request
import uuid


class SeekVLNClient:
    def __init__(self, endpoint="http://127.0.0.1:18012", timeout_s=120.0):
        self.endpoint = endpoint.rstrip("/")
        self.timeout_s = timeout_s

    def navigate(self, phase, instruction, frames, mode=None, views=None):
        files = list(frames) + list(views or [])
        if not files:
            raise ValueError("request has no frames")
        if phase == "mode" and len(files) != len(frames):
            raise ValueError("mode request cannot contain auxiliary views")
        if phase == "action" and mode == "seek" and len(views or []) != 3:
            raise ValueError("seek action requires left, forward and right views")
        request_id = uuid.uuid4().hex
        metadata = {"schema": "elf.seekvln-request.v1", "request_id": request_id,
                    "phase": phase, "instruction": instruction, "mode": mode,
                    "num_history_frames": len(frames), "num_aux_views": len(views or []),
                    "frames": [{"name": "frame_%03d" % index,
                                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                               for index, path in enumerate(files)]}
        boundary = ("----elf-seekvln-" + request_id).encode("ascii")
        body = []
        def field(name, value, content_type="application/json"):
            body.extend([b"--" + boundary + b"\r\n",
                         ("Content-Disposition: form-data; name=\"%s\"\r\n" % name).encode(),
                         ("Content-Type: %s\r\n\r\n" % content_type).encode(), value, b"\r\n"])
        field("metadata", json.dumps(metadata, separators=(",", ":")).encode())
        for index, path in enumerate(files):
            body.extend([b"--" + boundary + b"\r\n",
                         ("Content-Disposition: form-data; name=\"frame_%03d\"; filename=\"%s\"\r\n" %
                          (index, path.name)).encode(), b"Content-Type: image/jpeg\r\n\r\n",
                         path.read_bytes(), b"\r\n"])
        body.append(b"--" + boundary + b"--\r\n")
        request = urllib.request.Request(self.endpoint + "/v1/navigate", data=b"".join(body), method="POST",
                                         headers={"Content-Type": "multipart/form-data; boundary=" + boundary.decode()})
        started = time.monotonic()
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            result = json.loads(response.read().decode("utf-8"))
        if not isinstance(result, dict) or result.get("request_id") != request_id:
            raise RuntimeError("invalid inference response request_id")
        result["client_latency_s"] = time.monotonic() - started
        return result
