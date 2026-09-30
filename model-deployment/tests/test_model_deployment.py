import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from PIL import Image

from elf_model_deployment.actions import parse_seekvln_output, parse_seekvln_prediction
from elf_model_deployment.registry import registered_models
from elf_model_deployment.sampling import (MODEL_SENSOR_SIZE, materialize_seekvln_frames,
                                           prepare_model_frame, sample_indices)
from elf_model_deployment.contracts import SubgoalRequest
from elf_model_deployment.runner import execute_subgoal


def write_jpeg(path, size, color):
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.new("RGB", size, color).save(path, "JPEG")
    return path


class DeploymentTests(unittest.TestCase):
    def test_registry_and_sampling(self):
        self.assertEqual(registered_models(), ("navila", "seekvln-4k-sft"))
        self.assertEqual(sample_indices(12), [0, 1, 3, 4, 6, 7, 8, 10, 11])

    def test_short_history_has_no_padding(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); paths = []
            for index in range(3):
                paths.append(write_jpeg(root / ("source-%d.jpg" % index), (1920, 1080), (index * 60, 0, 0)))
            outputs = materialize_seekvln_frames(paths, root / "frames")
            self.assertEqual(len(outputs), 3)
            for path in outputs:
                self.assertEqual(Image.open(path).size, MODEL_SENSOR_SIZE)

    def test_prepare_letterboxes_full_fov_to_square(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = write_jpeg(root / "raw.jpg", (1920, 1080), (10, 200, 30))
            target = prepare_model_frame(source, root / "prepared" / "frame.jpg")
            image = Image.open(target)
            self.assertEqual(image.size, MODEL_SENSOR_SIZE)
            # The 16:9 content fills the full width; the padding sits at the
            # top and bottom rows (black letterbox bars).
            top = image.crop((0, 0, MODEL_SENSOR_SIZE[0], 1))
            self.assertEqual(set(top.getdata()), {(0, 0, 0)})
            middle = image.getpixel((MODEL_SENSOR_SIZE[0] // 2, MODEL_SENSOR_SIZE[1] // 2))
            self.assertTrue(all(abs(channel - 200) < 40 for channel in middle[1:2]))

    def test_strict_parser(self):
        self.assertEqual(parse_seekvln_output("The next action is forward 50 cm").value, 50)
        self.assertEqual(parse_seekvln_output("</seek><think>Completed tasks: none Next task: door Key Evidence: door </think><nav>turn left 15 degrees", "seek").name, "turn_left")
        self.assertEqual(parse_seekvln_output("turn left 25 degrees").name, "invalid")
        self.assertEqual(parse_seekvln_output("forward 25 cm and turn left 15 degrees").name, "invalid")
        self.assertEqual(parse_seekvln_prediction(
            {"format_valid": True, "actions": [2, 2, 2],
             "raw_text": "<nav>The next action is turn left 45 degree."}).value, 45)
        self.assertEqual(parse_seekvln_prediction(
            {"format_valid": True, "actions": [1, 2],
             "raw_text": "<nav>The next action is forward 50 cm."}).name, "invalid")

    def test_mock_nav_closed_loop_is_dry_run(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def observe(folder, index):
                path = write_jpeg(Path(folder) / ("observe-%d.jpg" % index), (1920, 1080), (0, 0, 0))
                return {"observation_id": str(index), "image": str(path), "observation": str(path)}, path
            class Client:
                def __init__(self, *_args, **_kwargs): pass
                def navigate(self, phase, *_args, **_kwargs):
                    if phase == "mode":
                        return {"mode": "nav", "scores": [1.0, 0.0]}
                    return {"raw_text": "<nav>move forward 25 cm", "request_id": "mock",
                            "format_valid": True, "actions": [1]}
            completed = type("Completed", (), {"returncode": 0, "stdout": "dry", "stderr": ""})
            with patch("elf_model_deployment.runner._observe", side_effect=observe), \
                 patch("elf_model_deployment.runner.SeekVLNClient", Client), \
                 patch("elf_model_deployment.runner._run", return_value=completed):
                result = execute_subgoal(SubgoalRequest(instruction="go", max_decisions=1),
                                         execute=False, run_dir=root)
            self.assertEqual(result.status, "budget_exhausted")
            self.assertEqual(result.decisions, 1)

    def test_seek_execute_controls_on_post_scan_observation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            observations = []
            def observe(folder, index):
                path = write_jpeg(Path(folder) / ("observe-%d.jpg" % index), (1920, 1080), (0, 0, 0))
                record = {"observation_id": "obs-%d" % len(observations), "image": str(path),
                          "observation": str(path) + ".json"}
                observations.append(record)
                return record, path
            class Client:
                def __init__(self, *_args, **_kwargs): pass
                def navigate(self, phase, *_args, **_kwargs):
                    if phase == "mode":
                        return {"mode": "seek", "scores": [0.0, 1.0]}
                    return {"raw_text": "</seek><think>Completed tasks: None. Next task: go. "
                                         "Key Evidence: open corridor. </think><nav>move forward 25 cm",
                            "request_id": "mock", "format_valid": True, "actions": [1]}
            def capture_views(directory, execute, alive=None):
                views = {}
                for name in ("left", "forward", "right"):
                    views[name] = write_jpeg(Path(directory) / (name + ".jpg"), (1920, 1080), (0, 90, 0))
                return views, {"dry_run": False}
            controlled = {}
            def fake_run(args, execute, observation=None, alive=None):
                controlled["observation"] = observation
                return type("Completed", (), {"returncode": 0, "stdout": "", "stderr": ""})()
            with patch("elf_model_deployment.runner._observe", side_effect=observe), \
                 patch("elf_model_deployment.runner.SeekVLNClient", Client), \
                 patch("elf_model_deployment.runner.capture_views", capture_views), \
                 patch("elf_model_deployment.runner._run", fake_run):
                result = execute_subgoal(SubgoalRequest(instruction="go", max_decisions=1,
                                                        max_forward_m=1.0),
                                         execute=True, run_dir=root)
            self.assertEqual(result.status, "budget_exhausted")
            self.assertEqual(len(observations), 3)  # start, post-scan, post-action
            self.assertEqual(controlled["observation"], observations[1]["observation"])


if __name__ == "__main__":
    unittest.main()
