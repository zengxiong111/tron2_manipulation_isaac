"""CPU-only tests for the pinned model metadata audit."""

from pathlib import Path
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

import numpy as np

from tron2_isaac.__main__ import audit_source_model, run_model_check
from tron2_isaac.model import MOTOR_NAMES, TORQUE_LIMITS


class ModelValidationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.controller = Path(self.temp.name) / "controller"
        self.asset = self.controller / "robot_description" / "tron2a" / "DACH_TRON2A"
        (self.asset / "urdf").mkdir(parents=True)
        (self.asset / "xml").mkdir()
        protocol = self.controller / "Tron2_arm_tests" / "dual_arm_protocol_tests"
        protocol.mkdir(parents=True)
        (protocol / "robot_model.py").write_text(
            "PROTOCOL_JOINT_NAMES = " + repr(list(MOTOR_NAMES)) + "\n"
        )
        self.urdf = self.asset / "urdf" / "robot_grasper.urdf"
        links = ['<link name="base_Link"/>']
        joints = []
        mjcf_joints = []
        for index, (name, effort) in enumerate(zip(MOTOR_NAMES, TORQUE_LIMITS)):
            child = f"link_{index}"
            links.append(f'<link name="{child}"/>')
            joints.append(
                f'<joint name="{name}" type="revolute">'
                '<parent link="base_Link"/>'
                f'<child link="{child}"/>'
                '<axis xyz="0 0 1"/>'
                f'<limit lower="-3.0" upper="3.0" effort="{effort}" velocity="2.0"/>'
                '</joint>'
            )
            mjcf_joints.append(
                f'<joint name="{name}" axis="0 0 1" range="-3.0 3.0" '
                f'actuatorfrcrange="{-effort} {effort}" armature="0.016"/>'
            )
        self.urdf.write_text('<robot name="fixture">' + ''.join(links + joints) + '</robot>')
        (self.asset / "xml" / "robot_grasper.xml").write_text(
            '<mujoco><compiler angle="radian"/><worldbody>'
            '<body name="base_Link" pos="0 0 1.21">' + ''.join(mjcf_joints)
            + '</body></worldbody></mujoco>'
        )

    def tearDown(self):
        self.temp.cleanup()

    def test_accepts_matching_order_axes_ranges_efforts_and_armatures(self):
        report = audit_source_model(self.urdf, self.controller)
        self.assertEqual(report["source_errors"], [])
        self.assertEqual(report["wire_order"], list(MOTOR_NAMES))
        self.assertEqual(len(report["motor_joints"]), 16)
        self.assertEqual(report["origin_audit"]["mjcf_authored_root_world_position_m"], [0.0, 0.0, 1.21])

    def test_rejects_wrong_signed_axis_and_range(self):
        mjcf = self.asset / "xml" / "robot_grasper.xml"
        content = mjcf.read_text()
        content = content.replace('axis="0 0 1" range="-3.0 3.0"', 'axis="0 0 -1" range="-4.0 3.0"', 1)
        mjcf.write_text(content)
        report = audit_source_model(self.urdf, self.controller)
        self.assertTrue(any("signed URDF/MuJoCo joint axes differ" in error for error in report["source_errors"]))
        self.assertTrue(any("position ranges differ" in error for error in report["source_errors"]))
        output = io.StringIO()
        with redirect_stdout(output):
            result = run_model_check(self.urdf, physics_rate=1000.0, controller_root=self.controller)
        self.assertEqual(result, 1)
        self.assertEqual(json.loads(output.getvalue())["model_check"], "failed")

    def test_imported_model_failure_raises_nonzero_after_json_report(self):
        report = audit_source_model(self.urdf, self.controller)
        armatures = [item["mjcf_armature_kg_m2"] for item in report["motor_joints"]]
        armatures[0] += 0.01
        limits = np.asarray([item["range_rad"] for item in report["motor_joints"]], dtype=float)
        efforts = np.asarray(TORQUE_LIMITS, dtype=float)

        class FakeView:
            def get_dof_limits(self):
                return limits[None, ...]

            def get_max_efforts(self, joint_indices=None):
                return efforts[None, ...]

            def get_armatures(self, joint_indices=None):
                return np.asarray(armatures, dtype=float)[None, ...]

        class FakeArticulation:
            dof_names = MOTOR_NAMES

            def __init__(self, parent):
                self._articulation_view = FakeView()
                self.parent = parent

            @staticmethod
            def get_dof_index(name):
                return MOTOR_NAMES.index(name)

            @staticmethod
            def get_world_pose():
                return np.zeros(3), np.asarray([0.0, 0.0, 0.0, 1.0])

            def get_applied_joint_efforts(self, joint_indices=None):
                return self.parent.last_effort

        class FakeRobot:
            def __init__(self, urdf, *, headless, rate):
                self._joint_indices = np.arange(len(MOTOR_NAMES), dtype=np.int32)
                self.last_effort = np.zeros(len(MOTOR_NAMES), dtype=float)
                self._robot = FakeArticulation(self)

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, traceback):
                return False

            @staticmethod
            def state():
                return np.asarray([0, 0.25, 0, -1.57, 0.236, 0, 0, 0, -0.25, 0, -1.57, -0.236, 0, 0, 0, 0]), np.zeros(16), np.zeros(16)

            def step(self, effort):
                self.last_effort = np.asarray(effort, dtype=float)

        output = io.StringIO()
        with patch("tron2_isaac.physics.IsaacRobot", FakeRobot), redirect_stdout(output):
            with self.assertRaises(SystemExit) as raised:
                run_model_check(self.urdf, physics_rate=1000.0, controller_root=self.controller)
        self.assertEqual(raised.exception.code, 1)
        self.assertEqual(json.loads(output.getvalue())["model_check"], "failed")


if __name__ == "__main__":
    unittest.main()
