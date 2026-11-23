"""MROS motor transport through LimX's ROS 2 bridge.

The Isaac Sim process can use a different Python ABI than ROS 2. This module
keeps ``rclpy`` in a system-Python sidecar and exchanges small JSON frames over
an inherited Unix socket. The official ``ros2-bridger`` process must already
be running in the same MROS network namespace.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import socket
import subprocess
import threading
from typing import Sequence

import numpy as np

from .model import MOTOR_NAMES


_BRIDGE_SCRIPT = Path(__file__).resolve().parents[1] / "bridge" / "mros_sidecar.py"
_ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
_FRAME_LIMIT = 64 * 1024


@dataclass(frozen=True)
class MotorCommand:
    """Latest controller command in the pinned DACH_TRON2A wire order."""

    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    servop_at: float = 0.0


class MrosTransport:
    """Bridge motor commands and simulated motor state.

    Args:
        bridge_install: The external LimX ``ros2-bridger`` install prefix,
            such as ``/opt/limxdynamics/ros2-bridger/amd64/jazzy/install``.
            No vendor bridge binaries are copied into this project.

    ``latest_command()`` returns the most recent ``/motor/cmd`` sample or
    ``None`` before one arrives. ``publish(q, dq, effort)`` sends current
    simulated state to ``/motor/state``; ``effort`` is written to ``tau``.
    The transport does not apply actuator dynamics or gains.
    """

    def __init__(self, bridge_install: Path | str) -> None:
        self.bridge_install = Path(bridge_install).expanduser().resolve()
        self._socket: socket.socket | None = None
        self._process: subprocess.Popen[bytes] | None = None
        self._reader: threading.Thread | None = None
        self._writer: threading.Thread | None = None
        self._state_condition = threading.Condition()
        self._pending_state: bytes | None = None
        self._state_lock = threading.Lock()
        self._latest: MotorCommand | None = None
        self._reader_error: BaseException | None = None
        self._ready = threading.Event()
        self._closed = threading.Event()

    def __enter__(self) -> "MrosTransport":
        setup = self.bridge_install / "setup.bash"
        if not _ROS_SETUP.is_file():
            raise FileNotFoundError(f"ROS 2 Jazzy setup not found: {_ROS_SETUP}")
        if not setup.is_file():
            raise FileNotFoundError(f"ros2-bridger setup not found: {setup}")
        if not _BRIDGE_SCRIPT.is_file():
            raise FileNotFoundError(f"ROS 2 sidecar not found: {_BRIDGE_SCRIPT}")

        parent_socket, child_socket = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        parent_socket.settimeout(1.0)
        child_fd = child_socket.fileno()
        shell_script = (
            'set -e; source "$1"; source "$2"; shift 2; '
            'exec "$@"'
        )
        python_executable = os.environ.get("TRON2_ROS2_PYTHON", "/usr/bin/python3")
        try:
            self._process = subprocess.Popen(
                [
                    "/bin/bash",
                    "-c",
                    shell_script,
                    "tron2-mros-sidecar",
                    str(_ROS_SETUP),
                    str(setup),
                    python_executable,
                    str(_BRIDGE_SCRIPT),
                    "--fd",
                    str(child_fd),
                ],
                pass_fds=(child_fd,),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=None,
                close_fds=True,
            )
        except BaseException:
            parent_socket.close()
            child_socket.close()
            raise
        child_socket.close()
        self._socket = parent_socket
        self._reader = threading.Thread(target=self._read_loop, name="mros-reader", daemon=True)
        self._reader.start()
        if not self._ready.wait(timeout=15.0):
            status = self._process.poll() if self._process else None
            self.close()
            if self._reader_error is not None:
                raise RuntimeError("MROS sidecar failed to start") from self._reader_error
            raise RuntimeError(f"MROS sidecar did not become ready (exit={status})")
        if self._reader_error is not None:
            error = self._reader_error
            self.close()
            raise RuntimeError("MROS sidecar failed to start") from error
        self._writer = threading.Thread(target=self._write_loop, name="mros-writer", daemon=True)
        self._writer.start()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def latest_command(self) -> MotorCommand | None:
        """Return the immutable latest command, without waiting for ROS."""

        if self._reader_error is not None:
            raise RuntimeError("MROS sidecar disconnected") from self._reader_error
        with self._state_lock:
            if self._latest is None:
                return None
            return self._latest

    def publish(
        self,
        q: Sequence[float],
        dq: Sequence[float],
        effort: Sequence[float],
    ) -> None:
        """Publish state vectors to the controller's ``/motor/state`` topic."""

        if self._socket is None or self._closed.is_set():
            raise RuntimeError("MROS transport is not connected")
        frame = {
            "type": "state",
            "q": _vector(q, "q"),
            "dq": _vector(dq, "dq"),
            "tau": _vector(effort, "effort"),
        }
        payload = json.dumps(frame, separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\n"
        if len(payload) > _FRAME_LIMIT:
            raise ValueError("MROS transport frame exceeds size limit")
        with self._state_condition:
            self._pending_state = payload
            self._state_condition.notify()

    def close(self) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        with self._state_condition:
            self._state_condition.notify_all()
        if self._socket is not None:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._socket.close()
            self._socket = None
        if self._process is not None:
            try:
                self._process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self._process.terminate()
                try:
                    self._process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait()
        if self._reader is not None and self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if self._writer is not None and self._writer.is_alive():
            self._writer.join(timeout=1.0)

    def _write_loop(self) -> None:
        assert self._socket is not None
        self._socket.settimeout(0.05)
        while not self._closed.is_set():
            with self._state_condition:
                while self._pending_state is None and not self._closed.is_set():
                    self._state_condition.wait(timeout=0.1)
                payload = self._pending_state
                self._pending_state = None
            if payload is None:
                continue
            try:
                self._socket.sendall(payload)
            except BaseException as exc:
                if not self._closed.is_set():
                    self._reader_error = exc
                return

    def _read_loop(self) -> None:
        assert self._socket is not None
        pending = bytearray()
        try:
            while not self._closed.is_set():
                try:
                    chunk = self._socket.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                pending.extend(chunk)
                if len(pending) > _FRAME_LIMIT:
                    raise ValueError("MROS sidecar frame exceeds size limit")
                while b"\n" in pending:
                    raw, _, remainder = pending.partition(b"\n")
                    pending = bytearray(remainder)
                    frame = json.loads(raw)
                    if frame.get("type") == "ready":
                        self._ready.set()
                    elif frame.get("type") == "command":
                        command = _command_from_frame(frame)
                        with self._state_lock:
                            self._latest = command
                    elif frame.get("type") == "error":
                        raise RuntimeError(str(frame.get("message", "unknown sidecar error")))
        except BaseException as exc:
            if not self._closed.is_set():
                self._reader_error = exc
                self._ready.set()
        finally:
            if not self._closed.is_set() and self._reader_error is None:
                self._reader_error = ConnectionError("MROS sidecar disconnected")
                self._ready.set()


def _vector(values: Sequence[float], name: str) -> list[float]:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (len(MOTOR_NAMES),):
        raise ValueError(f"{name} must contain {len(MOTOR_NAMES)} values, got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must contain finite values")
    return array.astype(np.float32).tolist()


def _command_from_frame(frame: dict[str, object]) -> MotorCommand:
    def read(name: str) -> np.ndarray:
        value = np.asarray(frame.get(name), dtype=np.float64)
        if value.shape != (len(MOTOR_NAMES),) or not np.isfinite(value).all():
            raise ValueError(f"invalid {name} received from MROS sidecar")
        return value

    values = [read(key) for key in ("q", "dq", "tau", "kp", "kd")]
    for value in values:
        value.setflags(write=False)
    servop_at = float(frame.get("servop_at", 0.0))
    if not np.isfinite(servop_at):
        raise ValueError("invalid ServoP timestamp received from MROS sidecar")
    return MotorCommand(*values, servop_at)
