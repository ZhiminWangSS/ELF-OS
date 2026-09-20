import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from elf_model_deployment.actions import parse_seekvln_output
from elf_model_deployment.registry import registered_models
from elf_model_deployment.sampling import materialize_seekvln_frames, sample_indices
from elf_model_deployment.contracts import SubgoalRequest
from elf_model_deployment.runner import execute_subgoal


class DeploymentTests(unittest.TestCase):
    def test_registry_and_sampling(self):
        self.assertEqual(registered_models(), ("navila", "seekvln-4k-sft"))
        self.assertEqual(sample_indices(12), [0, 1, 3, 4, 6, 7, 8, 10, 11])

    def test_short_history_has_no_padding(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); paths = []
            for index in range(3):
                path = root / ("source-%d.jpg" % index); path.write_bytes(bytes([index])); paths.append(path)
            outputs = materialize_seekvln_frames(paths, root / "frames")
            self.assertEqual(len(outputs), 3)
            self.assertEqual(outputs[-1].read_bytes(), b"\x02")

    def test_strict_parser(self):
        self.assertEqual(parse_seekvln_output("The next action is forward 50 cm").value, 50)
        self.assertEqual(parse_seekvln_output("</seek><think>Completed tasks: none Next task: door Key Evidence: door </think><nav>turn left 15 degrees", "seek").name, "turn_left")
        self.assertEqual(parse_seekvln_output("turn left 25 degrees").name, "invalid")
        self.assertEqual(parse_seekvln_output("forward 25 cm and turn left 15 degrees").name, "invalid")

    def test_mock_nav_closed_loop_is_dry_run(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            def observe(folder, index):
                path = Path(folder) / ("observe-%d.jpg" % index)
                path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(b"jpeg-fixture")
                return {"observation_id": str(index), "image": str(path), "observation": str(path)}, path
            class Client:
                def __init__(self, *_args, **_kwargs): pass
                def navigate(self, phase, *_args, **_kwargs):
                    if phase == "mode":
                        return {"mode": "nav", "scores": [1.0, 0.0]}
                    return {"raw_text": "move forward 25 cm", "request_id": "mock"}
            completed = type("Completed", (), {"returncode": 0, "stdout": "dry", "stderr": ""})
            with patch("elf_model_deployment.runner._observe", side_effect=observe), \
                 patch("elf_model_deployment.runner.SeekVLNClient", Client), \
                 patch("elf_model_deployment.runner._run", return_value=completed):
                result = execute_subgoal(SubgoalRequest(instruction="go", max_decisions=1),
                                         execute=False, run_dir=root)
            self.assertEqual(result.status, "budget_exhausted")
            self.assertEqual(result.decisions, 1)


if __name__ == "__main__":
    unittest.main()
