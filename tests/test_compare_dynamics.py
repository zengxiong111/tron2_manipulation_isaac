import unittest

from compare_dynamics import compare


class DynamicsComparisonTest(unittest.TestCase):
    def setUp(self):
        self.track = {
            "name": "servoJ", "start": [0.0], "target": [0.2], "rate_hz": 100.0,
            "duration_sec": 3.0, "hold_sec": 3.0, "tracking": {},
        }
        self.sample = {
            "slot": 0, "joint_t_sec": 0.0, "pose_t_sec": 0.0,
            "q": [0.0] * 16, "dq": [0.0] * 16, "tau": [0.0] * 16, "pose": [0.0] * 14,
        }

    def test_only_time_aligned_samples_are_compared(self):
        first = {"backend": "mujoco", "cases": [{**self.track, "samples": [
            self.sample, {**self.sample, "slot": 1, "joint_t_sec": 0.02, "pose_t_sec": 0.02},
        ]}]}
        second = {"backend": "isaac", "cases": [{**self.track, "samples": [
            {**self.sample, "q": [0.1] * 16, "pose": [0.001] + [0.0] * 13},
            {**self.sample, "slot": 1, "joint_t_sec": 0.05, "pose_t_sec": 0.05},
        ]}]}

        errors = compare(first, second)["cases"][0]["between_backends"]

        self.assertEqual(errors["q_rad"], {"samples": 1, "p95": 0.1, "max": 0.1})
        self.assertEqual(errors["position_mm"], {"samples": 1, "p95": 1.0, "max": 1.0})

    def test_mismatched_reference_is_rejected(self):
        first = {"backend": "mujoco", "cases": [self.track]}
        second = {"backend": "isaac", "cases": [{**self.track, "target": [0.1]}]}

        with self.assertRaisesRegex(ValueError, "same reference inputs"):
            compare(first, second)
