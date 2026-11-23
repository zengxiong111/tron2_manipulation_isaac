import unittest

import numpy as np

from tron2_isaac.__main__ import servop_gravity_axes
from tron2_isaac.mros import MotorCommand


class ServoPGravityAxesTest(unittest.TestCase):
    def test_expired_servop_does_not_receive_feedforward(self):
        zeros = np.zeros(16)
        command = MotorCommand(zeros, zeros, zeros, np.ones(16), zeros, 99.8)
        self.assertFalse(servop_gravity_axes(command, now=100.0).any())

    def test_missing_servop_does_not_receive_feedforward(self):
        zeros = np.zeros(16)
        command = MotorCommand(zeros, zeros, zeros, np.ones(16), zeros)
        self.assertFalse(servop_gravity_axes(command, now=0.05).any())

    def test_during_servop_the_head_keeps_controller_effort(self):
        zeros = np.zeros(16)
        command = MotorCommand(zeros, zeros, zeros, np.ones(16), zeros, 99.99)

        np.testing.assert_array_equal(
            servop_gravity_axes(command, now=100.0), [True] * 14 + [False] * 2
        )

    def test_existing_feedforward_and_unpowered_axes_are_excluded(self):
        zeros = np.zeros(16)
        feedforward = np.zeros(16)
        feedforward[0] = -7.0
        kp = np.ones(16)
        kp[7] = 0.0
        command = MotorCommand(zeros, zeros, feedforward, kp, zeros, 99.99)

        np.testing.assert_array_equal(
            servop_gravity_axes(command, now=100.0), [False] * 8 + [True] * 6 + [False] * 2
        )
