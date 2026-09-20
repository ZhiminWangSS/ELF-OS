from pathlib import Path
from typing import List, Sequence


def sample_indices(count: int, max_frames: int = 9):
    if count < 1 or max_frames != 9:
        raise ValueError("SeekVLN requires a non-empty history and max_frames=9")
    if count <= max_frames:
        return list(range(count))
    step = (count - 1) / float(max_frames - 1)
    return [int(round(i * step)) for i in range(max_frames)]


def materialize_seekvln_frames(paths: Sequence[Path], output: Path, max_frames: int = 9) -> List[Path]:
    """Copy real frames only; SeekVLN does not use temporal black padding."""
    if not paths:
        raise ValueError("at least one frame is required")
    output.mkdir(parents=True, exist_ok=True)
    for old in output.glob("frame_*.jpg"):
        old.unlink()
    result = []
    for out_index, source_index in enumerate(sample_indices(len(paths), max_frames)):
        target = output / ("frame_%03d.jpg" % out_index)
        target.write_bytes(Path(paths[source_index]).read_bytes())
        result.append(target)
    return result
