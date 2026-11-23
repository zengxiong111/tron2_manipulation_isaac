"""Run the Isaac physics backend by itself or with the LimX controller."""

import argparse
import ast
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from .model import ASSET_ROOT, HOME_Q, MOTOR_NAMES, TORQUE_LIMITS, motor_effort


DEFAULT_RATE = 300.0
MAX_PHYSICS_DT = 0.001
SERVOP_FRESHNESS_SEC = 0.1


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


def audit_source_model(urdf: Path, controller_root: Path | None = None) -> dict:
    """Compare pinned URDF and MuJoCo metadata without importing Isaac Sim."""
    urdf = Path(urdf).resolve()
    if not urdf.is_file():
        raise FileNotFoundError(urdf)
    mjcf = urdf.parent.parent / "xml" / f"{urdf.stem}.xml"
    if not mjcf.is_file():
        raise FileNotFoundError(mjcf)
    urdf_root = ET.parse(urdf).getroot()
    mjcf_root = ET.parse(mjcf).getroot()
    controller_root = Path(controller_root).resolve() if controller_root is not None else urdf.parents[4]
    protocol_model = controller_root / "Tron2_arm_tests" / "dual_arm_protocol_tests" / "robot_model.py"
    source_wire_order = None
    source_errors = []
    if protocol_model.is_file():
        parsed = ast.parse(protocol_model.read_text(encoding="utf-8"), filename=str(protocol_model))
        for node in parsed.body:
            if isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id == "PROTOCOL_JOINT_NAMES"
                for target in node.targets
            ):
                source_wire_order = ast.literal_eval(node.value)
                break
        if source_wire_order is None:
            source_errors.append(f"{protocol_model} has no literal PROTOCOL_JOINT_NAMES")
        elif tuple(source_wire_order) != MOTOR_NAMES:
            source_errors.append("motor wire order differs from pinned LimX PROTOCOL_JOINT_NAMES")
    else:
        source_errors.append(f"pinned LimX protocol model is missing: {protocol_model}")
    urdf_joints = {joint.get("name"): joint for joint in urdf_root.findall("joint")}
    mjcf_joints = {
        joint.get("name"): joint
        for joint in mjcf_root.iter("joint")
        if joint.get("name")
    }
    compiler = mjcf_root.find("compiler")
    angle_unit = compiler.get("angle", "radian") if compiler is not None else "radian"
    if angle_unit != "radian":
        raise ValueError(f"MuJoCo source angle unit is {angle_unit!r}, expected 'radian'")
    robot_desc = {"angle_unit": "radian", "position_unit": "meter", "effort_unit": "newton_meter"}
    meshes = urdf_root.findall(".//mesh")
    mesh_root = urdf.parent.parent / "meshes"
    missing_meshes = []
    for mesh in meshes:
        filename = mesh.get("filename", "")
        candidate = mesh_root / filename.rsplit("/", 1)[-1] if filename.startswith("package://") else (urdf.parent / filename).resolve()
        if not candidate.is_file():
            missing_meshes.append({"reference": filename, "resolved": str(candidate)})

    entries = []
    errors = source_errors
    for index, name in enumerate(MOTOR_NAMES):
        uj, mj = urdf_joints.get(name), mjcf_joints.get(name)
        if uj is None or mj is None:
            errors.append(f"missing motor joint {name} in {'URDF' if uj is None else 'MuJoCo'}")
            continue
        if uj.get("type") != "revolute":
            errors.append(f"{name}: URDF type is {uj.get('type')!r}, expected revolute")
        u_axis_el = uj.find("axis")
        u_limit = uj.find("limit")
        if u_axis_el is None or u_limit is None:
            errors.append(f"{name}: URDF axis or limit missing")
            continue
        u_axis = [float(x) for x in u_axis_el.get("xyz", "").split()]
        m_axis = [float(x) for x in mj.get("axis", "").split()]
        u_range = [float(u_limit.get("lower")), float(u_limit.get("upper"))]
        m_range = [float(x) for x in mj.get("range", "").split()]
        m_effort = [float(x) for x in mj.get("actuatorfrcrange", "").split()]
        u_effort = float(u_limit.get("effort", "nan"))
        armature = float(mj.get("armature", "nan"))
        if len(u_axis) != 3 or len(m_axis) != 3 or not np.allclose(u_axis, m_axis, atol=1e-6, rtol=0):
            errors.append(f"{name}: signed URDF/MuJoCo joint axes differ")
        if len(m_range) != 2 or not np.allclose(u_range, m_range, atol=1e-5, rtol=0):
            errors.append(f"{name}: URDF/MuJoCo position ranges differ")
        if len(m_effort) != 2 or not np.allclose(m_effort, [-u_effort, u_effort], atol=1e-5, rtol=0):
            errors.append(f"{name}: URDF effort and MuJoCo actuator force range differ")
        if not np.isclose(u_effort, TORQUE_LIMITS[index], atol=1e-6):
            errors.append(f"{name}: source effort limit differs from the motor wire limit")
        if not (u_range[0] <= HOME_Q[index] <= u_range[1]):
            errors.append(f"{name}: configured home position is outside source limits")
        entries.append({
            "name": name,
            "wire_index": index,
            "urdf_signed_axis": u_axis,
            "range_rad": u_range,
            "effort_limit_nm": u_effort,
            "mjcf_armature_kg_m2": armature,
        })

    root_body = mjcf_root.find("./worldbody/body")
    source_root_position = [float(x) for x in root_body.get("pos", "0 0 0").split()] if root_body is not None else None
    child_links = {joint.find("child").get("link") for joint in urdf_root.findall("joint") if joint.find("child") is not None}
    urdf_root_link = next((link.get("name") for link in urdf_root.findall("link") if link.get("name") not in child_links), None)
    if len(entries) != len(MOTOR_NAMES):
        errors.append("source model does not define all 16 motor joints")
    return {
        "urdf": str(urdf),
        "mjcf": str(mjcf),
        "wire_order": list(MOTOR_NAMES),
        "upstream_protocol_order": source_wire_order,
        "units": robot_desc,
        "motor_joints": entries,
        "mesh_count": len(meshes),
        "missing_meshes": missing_meshes,
        "origin_audit": {
            "urdf_root_link": urdf_root_link,
            "urdf_root_frame_origin_m": [0.0, 0.0, 0.0],
            "mjcf_root_link": root_body.get("name") if root_body is not None else None,
            "mjcf_authored_root_world_position_m": source_root_position,
        },
        "source_errors": errors + ([f"unresolved mesh references: {missing_meshes}"] if missing_meshes else []),
    }


def run_model_check(urdf: Path, physics_rate: float, controller_root: Path) -> int:
    """Run source checks and confirm the imported Isaac articulation metadata."""
    report = audit_source_model(urdf, controller_root)
    if report["source_errors"]:
        print(json.dumps({"model_check": "failed", **report}, indent=2), flush=True)
        return 1
    from .physics import IsaacRobot

    try:
        with IsaacRobot(urdf, headless=True, rate=physics_rate) as robot:
            articulation = robot._robot
            names = tuple(articulation.dof_names)
            indices = np.asarray([articulation.get_dof_index(name) for name in MOTOR_NAMES], dtype=np.int32)
            applied_indices = np.asarray(robot._joint_indices, dtype=np.int32)
            view = articulation._articulation_view
            limits = np.asarray(view.get_dof_limits())[0, indices, :]
            efforts = np.asarray(view.get_max_efforts(joint_indices=indices))[0]
            armatures = np.asarray(view.get_armatures(joint_indices=indices))[0]
            q, dq, measured_effort = robot.state()
            expected_limits = np.asarray([item["range_rad"] for item in report["motor_joints"]])
            expected_efforts = np.asarray(TORQUE_LIMITS, dtype=np.float64)
            expected_armatures = np.asarray([item["mjcf_armature_kg_m2"] for item in report["motor_joints"]])
            root_position, _ = articulation.get_world_pose()
            root_position = np.asarray(root_position, dtype=float).tolist()
            effort_probe = np.zeros(len(MOTOR_NAMES), dtype=np.float32)
            effort_probe[-1] = 0.5
            robot.step(effort_probe)
            _, _, probe_solver_effort = robot.state()
            probe_applied_effort = np.asarray(
                articulation.get_applied_joint_efforts(joint_indices=indices), dtype=np.float64
            )
            checks = {
                "exact_16_dofs": len(names) == len(MOTOR_NAMES) and set(names) == set(MOTOR_NAMES),
                "wire_index_map": bool(
                    np.array_equal(indices, applied_indices)
                    and indices.tolist() == [names.index(name) for name in MOTOR_NAMES]
                ),
                "authored_limits_match": bool(np.allclose(limits, expected_limits, atol=1e-5, rtol=0)),
                "effort_limits_match": bool(np.allclose(efforts, expected_efforts, atol=1e-4, rtol=0)),
                "armatures_match": bool(np.allclose(armatures, expected_armatures, atol=1e-6, rtol=1e-5)),
                "home_positions_match": bool(np.allclose(q, HOME_Q, atol=1e-5, rtol=0)),
                "finite_initial_state": bool(np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(measured_effort).all()),
                "effort_probe_observed": bool(np.isclose(probe_applied_effort[-1], effort_probe[-1], atol=1e-5, rtol=0)),
                "effort_probe_within_joint_limit": bool(np.all(np.abs(probe_applied_effort) <= efforts + 1e-5)),
            }
            report["isaac"] = {
                "dof_names_in_importer_order": list(names),
                "wire_to_dof_indices": indices.tolist(),
                "applied_motor_indices": applied_indices.tolist(),
                "dof_limits_rad": limits.tolist(),
                "max_efforts_nm": efforts.tolist(),
                "armatures_kg_m2": armatures.tolist(),
                "initial_q_rad": q.tolist(),
                "initial_dq_rad_s": dq.tolist(),
                "effort_probe_requested_nm": effort_probe.tolist(),
                "effort_probe_applied_nm": probe_applied_effort.tolist(),
                "effort_probe_solver_measured_nm": probe_solver_effort.tolist(),
                "root_world_position_m": root_position,
            }
            report["origin_audit"]["same_common_world_origin"] = bool(
                np.allclose(root_position, report["origin_audit"]["mjcf_authored_root_world_position_m"], atol=1e-5, rtol=0)
            )
            report["origin_audit"]["isaac_root_world_position_m"] = root_position
            report["origin_audit"]["isaac_minus_mjcf_root_m"] = (
                np.asarray(root_position, dtype=float)
                - np.asarray(report["origin_audit"]["mjcf_authored_root_world_position_m"], dtype=float)
            ).tolist()
            report["origin_audit"]["layout_status"] = (
                "aligned" if report["origin_audit"]["same_common_world_origin"] else "reported_difference"
            )
            report["checks"] = checks
            passed = all(checks.values())
            report["model_check"] = "passed" if passed else "failed"
            print(json.dumps(report, indent=2), flush=True)
            if not passed:
                raise SystemExit(1)
            return 0
    except Exception as exc:
        report["model_check"] = "failed"
        report["runtime_error"] = f"{type(exc).__name__}: {exc}"
        print(json.dumps(report, indent=2), flush=True)
        return 1


def servop_gravity_axes(command, now: float) -> np.ndarray:
    """Select the active ServoP arms for which LimX supplies no feedforward."""
    if command.servop_at <= 0.0 or not 0.0 <= now - command.servop_at <= SERVOP_FRESHNESS_SEC:
        return np.zeros(len(MOTOR_NAMES), dtype=bool)
    has_feedforward = np.any(command.tau[:14].reshape(2, 7) != 0.0, axis=1)
    arm_axes = np.repeat(~has_feedforward, 7)
    return np.concatenate((arm_axes, (False, False))) & (command.kp > 0.0)


def run_controller(
    robot, transport, rate: float, substeps: int, duration: float | None,
    servop_gravity_compensation: bool,
) -> None:
    dt = 1.0 / rate
    started = deadline = time.monotonic()
    state_frames = 0
    command_steps = 0
    compensating = False
    gravity_steps = 0
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
                axes = servop_gravity_axes(command, time.monotonic()) if servop_gravity_compensation else None
                active = axes is not None and bool(axes.any())
                if active != compensating:
                    print(f"ServoP gravity compensation {'active' if active else 'inactive'}", flush=True)
                    compensating = active
                feedforward = command.tau
                if active:
                    feedforward = feedforward + np.where(axes, robot.gravity_effort(), 0.0)
                    gravity_steps += 1
                for index in range(substeps):
                    effort = motor_effort(q, dq, command.q, command.dq, feedforward, command.kp, command.kd)
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
            "steps_with_servop_gravity_compensation": gravity_steps,
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
    parser.add_argument("--check-model", action="store_true", help="Audit source metadata and verify the imported Isaac articulation without ROS")
    parser.add_argument(
        "--no-servop-gravity-compensation", action="store_true",
        help="Use only LimX motor feedforward, including during ServoP",
    )
    args = parser.parse_args()
    for name, value in (("--rate", args.rate), ("--seconds", args.seconds)):
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f"{name} must be finite and positive")
    upstream = args.upstream.resolve()
    urdf = upstream / ASSET_ROOT / "urdf/robot_grasper.urdf"
    if not urdf.is_file():
        parser.error(f"DACH_TRON2A URDF missing: {urdf}. Run scripts/get_upstream.sh first.")
    if args.check_model and args.smoke:
        parser.error("--check-model and --smoke are separate Isaac checks")
    if args.check_model:
        substeps = max(1, math.ceil(1 / (MAX_PHYSICS_DT * args.rate)))
        raise SystemExit(run_model_check(urdf, args.rate * substeps, upstream))
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
                    run_controller(
                        robot, transport, args.rate, substeps, args.seconds,
                        not args.no_servop_gravity_compensation,
                    )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
