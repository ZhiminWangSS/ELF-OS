import hashlib
import json
import mimetypes
import time
import urllib.request
import uuid


class InferenceClient:
    def __init__(self, endpoint="http://127.0.0.1:18011", timeout_s=120.0):
        self.endpoint = endpoint.rstrip("/")
        self.timeout_s = timeout_s

    def navigate(self, instruction, frame_paths):
        if not frame_paths:
            raise ValueError("frame_paths must not be empty")
        boundary = ("----elf-model-" + uuid.uuid4().hex).encode("ascii")
        request_id = uuid.uuid4().hex
        metadata = json.dumps({"schema": "elf.navigate-request.v1", "request_id": request_id,
                               "instruction": instruction, "num_video_frames": len(frame_paths),
                               "frames": [{"name": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                                          for p in frame_paths]}, separators=(",", ":")).encode("utf-8")
        body = []
        def field(name, value, content_type="application/json"):
            body.extend([b"--" + boundary + b"\r\n", ("Content-Disposition: form-data; name=\"%s\"\r\n" % name).encode(),
                         ("Content-Type: %s\r\n\r\n" % content_type).encode(), value, b"\r\n"])
        field("metadata", metadata)
        for index, path in enumerate(frame_paths):
            data = path.read_bytes()
            body.extend([b"--" + boundary + b"\r\n", ("Content-Disposition: form-data; name=\"frame_%03d\"; filename=\"%s\"\r\n" % (index, path.name)).encode(),
                         b"Content-Type: image/jpeg\r\n\r\n", data, b"\r\n"])
        body.append(b"--" + boundary + b"--\r\n")
        request = urllib.request.Request(self.endpoint + "/v1/navigate", data=b"".join(body), method="POST",
                                         headers={"Content-Type": "multipart/form-data; boundary=" + boundary.decode()})
        started = time.monotonic()
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            result = json.loads(response.read().decode("utf-8"))
        if not isinstance(result.get("action"), str):
            raise RuntimeError("inference response has no action")
        result["client_latency_s"] = time.monotonic() - started
        return result
