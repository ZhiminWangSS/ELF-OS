import hashlib
import io
import json


MAX_FRAMES = 12
MAX_FRAME_BYTES = 8 * 1024 * 1024


def validate_request(metadata, parts):
    if metadata.get("schema") != "elf.seekvln-request.v1":
        raise ValueError("unsupported request schema")
    request_id = metadata.get("request_id")
    if not isinstance(request_id, str) or not request_id:
        raise ValueError("request_id is required")
    if metadata.get("phase") not in ("mode", "action"):
        raise ValueError("phase must be mode or action")
    if not isinstance(metadata.get("instruction"), str) or not metadata["instruction"].strip():
        raise ValueError("instruction is required")
    names = [item.get("name") for item in metadata.get("frames", [])]
    if not names or len(names) > MAX_FRAMES or names != sorted(names):
        raise ValueError("invalid ordered frame manifest")
    if set(names) != set(parts):
        raise ValueError("uploaded frames do not match manifest")
    if metadata["phase"] == "mode" and len(names) != int(metadata.get("num_history_frames", -1)):
        raise ValueError("mode frame count mismatch")
    if metadata["phase"] == "action":
        mode = metadata.get("mode")
        expected = int(metadata.get("num_history_frames", -1)) + (3 if mode == "seek" else 0)
        if mode not in ("nav", "seek") or len(names) != expected:
            raise ValueError("action frame count/mode mismatch")
    try:
        from PIL import Image
        for item in metadata["frames"]:
            if not isinstance(item, dict) or not isinstance(item.get("name"), str):
                raise ValueError("invalid frame manifest entry")
            data = parts[item["name"]]
            if len(data) == 0 or len(data) > MAX_FRAME_BYTES:
                raise ValueError("invalid JPEG size")
            if hashlib.sha256(data).hexdigest() != item.get("sha256"):
                raise ValueError("frame hash mismatch")
            with Image.open(io.BytesIO(data)) as image:
                if image.format != "JPEG":
                    raise ValueError("frame is not JPEG")
                image.verify()
    except ImportError:
        raise RuntimeError("Pillow is required by the inference service")
    return names


def parse_metadata(raw):
    try:
        value = json.loads(raw.decode("utf-8") if isinstance(raw, bytes) else raw)
    except (TypeError, ValueError) as exc:
        raise ValueError("metadata is not valid JSON") from exc
    if not isinstance(value, dict):
        raise ValueError("metadata must be an object")
    return value
