"""DACH_TRON2A motor wire order and limits for the pinned LimX assets."""

from pathlib import Path

import numpy as np


CONTROLLER_REV = "10af63c20e0dd2149168c6693453cc28f2c2dcbf"
DESCRIPTION_REV = "9939c22e69d27653ec0ba8a505859a2903dd1a71"
ASSET_ROOT = Path("robot_description/tron2a/DACH_TRON2A")

MOTOR_NAMES = (
    "proximal_pitch_L_Joint",
    "proximal_roll_L_Joint",
    "proximal_yaw_L_Joint",
    "elbow_L_Joint",
    "wrist_yaw_L_Joint",
    "wrist_pitch_L_Joint",
    "wrist_roll_L_Joint",
    "proximal_pitch_R_Joint",
    "proximal_roll_R_Joint",
    "proximal_yaw_R_Joint",
    "elbow_R_Joint",
    "wrist_yaw_R_Joint",
    "wrist_pitch_R_Joint",
    "wrist_roll_R_Joint",
    "head_pitch_Joint",
    "head_yaw_Joint",
)
HOME_Q = (0, 0.25, 0, -1.57, 0.236, 0, 0, 0, -0.25, 0, -1.57, -0.236, 0, 0, 0, 0)
TORQUE_LIMITS = (150, 150, 60, 70, 20, 20, 20) * 2 + (20, 20)


def motor_effort(
    q: np.ndarray,
    dq: np.ndarray,
    target_q: np.ndarray,
    target_dq: np.ndarray,
    feedforward: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
) -> np.ndarray:
    effort = feedforward + kp * (target_q - q) + kd * (target_dq - dq)
    if effort.shape != (len(MOTOR_NAMES),) or not np.isfinite(effort).all():
        raise ValueError("Motor effort must be 16 finite values")
    return np.clip(effort, -np.asarray(TORQUE_LIMITS), TORQUE_LIMITS)
