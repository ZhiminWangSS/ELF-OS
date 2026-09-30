import math
from pathlib import Path
from typing import List, Sequence

from PIL import Image


# The Go2 front camera (videohub service) publishes 16:9 frames only (1080p or
# 720p; the SDK exposes no square or 4:3 mode). To keep the full wide FOV
# while giving the model its square training-time geometry, frames are
# uniformly scaled into a square canvas and letterboxed vertically.
MODEL_SENSOR_SIZE = (512, 512)
JPEG_QUALITY = 90


def prepare_model_frame(source: Path, target: Path) -> Path:
    """Full-FOV letterbox: uniform scale into a square, pad the remainder."""
    image = Image.open(source).convert("RGB")
    width, height = image.size
    scale = min(MODEL_SENSOR_SIZE[0] / width, MODEL_SENSOR_SIZE[1] / height)
    resized = image.resize((max(1, round(width * scale)), max(1, round(height * scale))), Image.BICUBIC)
    canvas = Image.new("RGB", MODEL_SENSOR_SIZE, (0, 0, 0))
    canvas.paste(resized, ((MODEL_SENSOR_SIZE[0] - resized.width) // 2,
                           (MODEL_SENSOR_SIZE[1] - resized.height) // 2))
    target.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(target, "JPEG", quality=JPEG_QUALITY)
    return target


def sample_indices(count: int, max_frames: int = 9):
    if count < 1 or max_frames != 9:
        raise ValueError("SeekVLN requires a non-empty history and max_frames=9")
    if count <= max_frames:
        return list(range(count))
    step = (count - 1) / float(max_frames - 1)
    return [int(round(i * step)) for i in range(max_frames)]


def materialize_seekvln_frames(paths: Sequence[Path], output: Path, max_frames: int = 9) -> List[Path]:
    """Prepare real frames only; SeekVLN does not use temporal black padding."""
    if not paths:
        raise ValueError("at least one frame is required")
    output.mkdir(parents=True, exist_ok=True)
    for old in output.glob("frame_*.jpg"):
        old.unlink()
    result = []
    for out_index, source_index in enumerate(sample_indices(len(paths), max_frames)):
        target = output / ("frame_%03d.jpg" % out_index)
        prepare_model_frame(Path(paths[source_index]), target)
        result.append(target)
    return result
