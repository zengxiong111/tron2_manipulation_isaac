from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from tron2_isaac.sdk import resolve_states_files


class SdkPathsTest(unittest.TestCase):
    def test_vla_states_file_can_be_found_in_the_pinned_sdk(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "examples" / "vla_height" / "states_controller_safe.txt"
            path.parent.mkdir(parents=True)
            path.touch()
            cases = resolve_states_files([{"states_file": str(path.relative_to(root))}], root)

            self.assertEqual(cases[0]["states_file"], str(path))

    def test_absolute_and_missing_paths_pass_through(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            expected = [{"states_file": str(root / "external.txt")}, {"states_file": "missing.txt"}, {}]

            self.assertEqual(resolve_states_files([case.copy() for case in expected], root), expected)
