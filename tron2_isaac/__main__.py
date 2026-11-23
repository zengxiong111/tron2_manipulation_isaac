"""Run the Isaac physics backend by itself or with the LimX controller."""

import argparse
import json
import math
from pathlib import Path
import time

import numpy as np

from .model import ASSET_ROOT, MOTOR_NAMES, motor_effort


DEFAULT_RATE = 300.0
MAX_PHYSICS_DT = 0.001


def run_physics_smoke(robot, rate: float) -> None:
    initial, _, _ = robot.state()
    initial = np.array(initial, copy=True)
    target = initial.copy()
    axes = [0, 7, 14, 15]
    target[axes] += [0.12, -0.12, 0.12, -0.12]
    kp = np.array([160, 120, 60, 120, 45, 45, 45] * 2 + [35, 35])
    kd = np.array([16, 14, 8, 15, 5, 5, 5] * 2 + [4, 4])
    zeros = np.zeros(len(MOTOR_NAMES))
    peak_effort = 0.0
    for _ in range(round(4.0 * rate)):
        q, dq, _ = robot.state()
        effort = motor_effort(q, dq, target, zeros, zeros, kp, kd)
        peak_effort = max(peak_effort, float(np.max(np.abs(effort))))
        robot.step(effort)
    q, dq, _ = robot.state()
    error = np.abs(q[axes] - target[axes])
    max_speed = float(np.max(np.abs(dq[axes])))
    tracked = bool(
        np.isfinite(q).all()
        and np.isfinite(dq).all()
        and np.all(error < 0.05)
        and max_speed < 0.1
        and peak_effort > 0
    )
    print(json.dumps({
        "physics_smoke": "passed" if tracked else "failed",
        "motor_count": len(q),
        "axes": [MOTOR_NAMES[i] for i in axes],
        "target_rad": target[axes].tolist(),
        "measured_rad": q[axes].tolist(),
        "max_error_rad": float(np.max(error)),
        "max_speed_rad_s": max_speed,
        "peak_effort_nm": peak_effort,
    }, indent=2), flush=True)
    if not tracked:
        raise SystemExit(1)


def run_controller(robot, transport, rate: float, substeps: int, duration: float | None) -> None:
    dt = 1.0 / rate
    started = deadline = time.monotonic()
    state_frames = 0
    command_steps = 0
    print(f"Isaac motor simulation ready: {rate:g} Hz motor loop, {rate * substeps:g} Hz PhysX", flush=True)
    try:
        while robot.is_running() and (duration is None or time.monotonic() - started < duration):
            q, dq, measured_effort = robot.state()
            transport.publish(q, dq, measured_effort)
            state_frames += 1
            command = transport.latest_command()
            if command is None:
                robot.render_idle()
            else:
                if not command_steps:
                    print("Received first /motor/cmd", flush=True)
                command_steps += 1
                for index in range(substeps):
                    effort = motor_effort(q, dq, command.q, command.dq, command.tau, command.kp, command.kd)
                    robot.step(effort)
                    if index + 1 < substeps:
                        q, dq, _ = robot.state()
            deadline = max(deadline + dt, time.monotonic())
            time.sleep(max(0.0, deadline - time.monotonic()))
    finally:
        elapsed = time.monotonic() - started
        print(json.dumps({
            "state_frames": state_frames,
            "steps_with_motor_command": command_steps,
            "state_rate_hz": state_frames / elapsed,
            "wall_seconds": elapsed,
        }), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", required=True, type=Path, help="Pinned LimX controller checkout")
    parser.add_argument("--bridge-install", type=Path, help="LimX ros2-bridger Jazzy install directory")
    parser.add_argument("--gui", action="store_true", help="Open the Isaac Sim window")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE, help="Motor state rate in Hz")
    parser.add_argument("--seconds", type=float, help="Optional controller run duration in wall-clock seconds")
    parser.add_argument("--smoke", action="store_true", help="Test PhysX joint tracking without the controller")
    args = parser.parse_args()
    for name, value in (("--rate", args.rate), ("--seconds", args.seconds)):
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f"{name} must be finite and positive")
    upstream = args.upstream.resolve()
    urdf = upstream / ASSET_ROOT / "urdf/robot_grasper.urdf"
    if not urdf.is_file():
        parser.error(f"DACH_TRON2A URDF missing: {urdf}. Run scripts/get_upstream.sh first.")
    if not args.smoke and (args.bridge_install is None or not (args.bridge_install / "setup.bash").is_file()):
        parser.error("Controller mode requires --bridge-install pointing to the Jazzy install directory")

    from .physics import IsaacRobot

    substeps = max(1, math.ceil(1 / (MAX_PHYSICS_DT * args.rate)))
    physics_rate = args.rate * substeps
    try:
        with IsaacRobot(urdf, headless=not args.gui, rate=physics_rate) as robot:
            if args.smoke:
                run_physics_smoke(robot, physics_rate)
            else:
                from .mros import MrosTransport

                with MrosTransport(args.bridge_install.resolve()) as transport:
                    run_controller(robot, transport, args.rate, substeps, args.seconds)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
