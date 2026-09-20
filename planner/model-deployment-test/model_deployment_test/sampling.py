from pathlib import Path
from typing import List, Sequence, Tuple


def sample_indices(count: int, num_frames: int = 8) -> Tuple[int, ...]:
    if count < 1 or num_frames < 1:
        raise ValueError("count and num_frames must be positive")
    effective = max(count, num_frames)
    history = num_frames - 1
    indices = tuple(int(i * (effective - 1) / history) for i in range(history)) if history else ()
    return indices + (effective - 1,)


def materialize_frames(paths: Sequence[Path], output: Path, num_frames: int = 8) -> List[Path]:
    if not paths:
        raise ValueError("at least one frame is required")
    output.mkdir(parents=True, exist_ok=True)
    for old in output.glob("frame_*.jpg"):
        old.unlink()
    padding = max(0, num_frames - len(paths))
    result = []
    for out_index, effective_index in enumerate(sample_indices(len(paths), num_frames)):
        source = effective_index - padding
        target = output / ("frame_%03d.jpg" % out_index)
        if source < 0:
            _black_frame(target)
        else:
            target.write_bytes(Path(paths[source]).read_bytes())
        result.append(target)
    return result


def _black_frame(path: Path) -> None:
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError("Pillow is required for frame padding") from exc
    Image.new("RGB", (448, 448), (0, 0, 0)).save(str(path), "JPEG", quality=95)
