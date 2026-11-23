#!/usr/bin/env python3
"""ROS 2 Jazzy bridge sidecar for the Isaac Python process."""

from __future__ import annotations

import argparse
import json
import math
import select
import socket
from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from controller_msgs.msg import JointCmd, JointState


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
MOTOR_COUNT = len(MOTOR_NAMES)
MAX_FRAME = 64 * 1024


def _ordered(values: Sequence[float], names: Sequence[str], label: str) -> list[float]:
    data = [float(value) for value in values]
    if not data:
        if label == "q":
            raise ValueError("command q array is empty")
        return [0.0] * MOTOR_COUNT
    if any(not math.isfinite(value) for value in data):
        raise ValueError(f"command {label} contains a non-finite value")
    if len(data) != MOTOR_COUNT:
        raise ValueError(f"command {label} has {len(data)} values, expected {MOTOR_COUNT}")
    if not names:
        return data
    if len(names) != MOTOR_COUNT or len(set(names)) != MOTOR_COUNT:
        raise ValueError("command names must be empty or contain 16 unique motor names")
    if set(names) != set(MOTOR_NAMES):
        unknown = sorted(set(names) - set(MOTOR_NAMES))
        missing = sorted(set(MOTOR_NAMES) - set(names))
        raise ValueError(f"command names differ from canonical motor names; unknown={unknown}, missing={missing}")
    indices = {name: index for index, name in enumerate(names)}
    return [data[indices[name]] for name in MOTOR_NAMES]


def _encode(frame: dict[str, object]) -> bytes:
    payload = json.dumps(frame, separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\n"
    if len(payload) > MAX_FRAME:
        raise ValueError("sidecar frame exceeds size limit")
    return payload


class BridgeNode(Node):
    def __init__(self, channel: socket.socket) -> None:
        super().__init__("tron2_isaac_mros_transport")
        self.channel = channel
        self.command_sequence = 0
        self.latest_command: dict[str, list[float]] | None = None
        self.tx = bytearray()
        self.pending_command: bytes | None = None
        self.queued_command: bytes | None = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(JointState, "/motor/state", qos)
        self.create_subscription(JointCmd, "/motor/cmd", self._on_command, qos)

    def _on_command(self, message: JointCmd) -> None:
        names = list(message.names)
        try:
            command = {
                "q": _ordered(message.q, names, "q"),
                "dq": _ordered(message.v, names, "v"),
                "tau": _ordered(message.tau, names, "tau"),
                "kp": _ordered(message.kp, names, "kp"),
                "kd": _ordered(message.kd, names, "kd"),
            }
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"ignoring malformed /motor/cmd: {exc}")
            return
        self.latest_command = command
        self.command_sequence += 1

    def receive_states(self) -> bool:
        ready, _, _ = select.select([self.channel], [], [], 0)
        if not ready:
            return True
        try:
            chunk = self.channel.recv(8192)
        except BlockingIOError:
            return True
        if not chunk:
            return False
        self.tx.extend(chunk)
        if len(self.tx) > MAX_FRAME:
            raise ValueError("state input frame exceeds size limit")
        while b"\n" in self.tx:
            raw, _, remainder = self.tx.partition(b"\n")
            self.tx = bytearray(remainder)
            frame = json.loads(raw)
            if frame.get("type") == "close":
                return False
            if frame.get("type") != "state":
                continue
            q = _validate_state_vector(frame.get("q"), "q")
            dq = _validate_state_vector(frame.get("dq"), "dq")
            tau = _validate_state_vector(frame.get("tau"), "tau")
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = ""
            message.names = list(MOTOR_NAMES)
            message.q = q
            message.v = dq
            message.vd = [0.0] * MOTOR_COUNT
            message.tau = tau
            message.na = MOTOR_COUNT
            self.state_pub.publish(message)
        return True

    def send_command(self) -> None:
        if self.latest_command is None or self.command_sequence == 0:
            return
        frame = {"type": "command", **self.latest_command}
        encoded = _encode(frame)
        if self.pending_command is None:
            self.pending_command = encoded
        else:
            self.queued_command = encoded
        self.command_sequence = 0

    def flush_command(self) -> None:
        if self.pending_command is None:
            return
        _, writable, _ = select.select([], [self.channel], [], 0)
        if not writable:
            return
        try:
            sent = self.channel.send(self.pending_command)
        except BlockingIOError:
            return
        self.pending_command = self.pending_command[sent:]
        if not self.pending_command:
            self.pending_command = self.queued_command
            self.queued_command = None


def _validate_state_vector(values: object, label: str) -> list[float]:
    if not isinstance(values, list) or len(values) != MOTOR_COUNT:
        raise ValueError(f"state {label} must contain {MOTOR_COUNT} values")
    result = [float(value) for value in values]
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"state {label} contains a non-finite value")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fd", type=int, required=True)
    args = parser.parse_args()
    channel = socket.socket(fileno=args.fd)
    rclpy.init(args=None)
    node = BridgeNode(channel)
    try:
        channel.sendall(_encode({"type": "ready"}))
        channel.setblocking(False)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.002)
            if not node.receive_states():
                break
            node.send_command()
            node.flush_command()
    except (BrokenPipeError, ConnectionError, OSError, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        channel.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
