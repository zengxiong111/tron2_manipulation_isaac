"""Capture matched TRON2 SDK tracks and compare MuJoCo and Isaac Sim traces."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import sys
import time


SAMPLE_HZ = 50
HOLD_SEC = 3.0
PAIR_TOLERANCE_SEC = 0.01


def sleep_until(deadline):
    time.sleep(max(0.0, deadline - time.monotonic()))


def summary(values):
    values = sorted(values)
    if not values:
        raise ValueError("The trace has no samples in the requested comparison window")
    return {"p95": values[math.ceil(0.95 * len(values)) - 1], "max": values[-1]}


def joint_distance(left, right):
    return max(abs(a - b) for a, b in zip(left, right, strict=True))


def position_distance(left, right):
    return 1000.0 * max(math.dist(left[:3], right[:3]), math.dist(left[7:10], right[7:10]))


def summarize_track(case, samples, start, target):
    duration = case["duration_sec"]
    field, distance = ("q", joint_distance) if case["name"] == "servoJ" else ("pose", position_distance)
    timestamp = "joint_t_sec" if field == "q" else "pose_t_sec"
    in_motion = [sample for sample in samples if 0 <= sample[timestamp] <= duration]
    errors = []
    for sample in in_motion:
        fraction = sample[timestamp] / duration
        desired = [a + fraction * (b - a) for a, b in zip(start, target, strict=True)]
        errors.append(distance(sample[field], desired))
    final = samples[-1]
    return {"following": summary(errors), "hold_end": distance(final[field], target),
            "units": "rad" if field == "q" else "mm"}


def capture(backend):
    import yaml

    if "TRON2_RUN_DIR" not in os.environ:
        raise RuntimeError("Start the isolated simulator before capturing live SDK data")
    sdk_root = Path(os.environ["TRON2_UPSTREAM"]).resolve() / "Tron2_arm_tests"
    sys.path.insert(0, str(sdk_root))
    from dual_arm_protocol_tests.client import DualArmProtocolClient
    from dual_arm_protocol_tests.motion import wait_for_joint_target, wait_for_pose_target
    from dual_arm_protocol_tests.trajectory import interpolate_pose, interpolate_vectors

    suite = yaml.safe_load((sdk_root / "config/test_cases.yaml").read_text())
    cases = {case["id"]: case for case in suite["cases"]}
    command = DualArmProtocolClient("127.0.0.1", accid="DACH_TRON2A_001")
    observer = DualArmProtocolClient("127.0.0.1", accid="DACH_TRON2A_001")

    def move(name, payload):
        reply = command.request(name, payload)
        if reply["data"].get("result") != "success":
            raise RuntimeError(f"Preparation command failed: {reply}")

    def observe(start_time, duration):
        samples = []
        for slot in range(round(duration * SAMPLE_HZ)):
            nominal = slot / SAMPLE_HZ
            sleep_until(start_time + nominal)
            before = time.monotonic()
            joints = observer.get_joint_state()
            after = time.monotonic()
            pose = observer.get_move_pose().as_flat_pose()
            pose_after = time.monotonic()
            if any(len(vector) != 16 for vector in (joints.q, joints.dq, joints.tau)) or len(pose) != 14:
                raise RuntimeError("Invalid joint state or endpoint dimensions from the SDK")
            samples.append({
                "slot": slot,
                "joint_t_sec": (before + after) / 2 - start_time,
                "pose_t_sec": (after + pose_after) / 2 - start_time,
                "joint_stamp_ms": joints.timestamp_ms,
                "q": joints.q, "dq": joints.dq, "tau": joints.tau, "pose": pose,
            })
        return samples

    def track(name, source, start, target):
        duration = float(source["duration"])
        rate = float(source["send_rate_hz"])
        if name == "servoJ":
            interpolate = interpolate_vectors
            title = "request_servoj"

            def payload(values):
                return {"q": values, "filter_ratio": source["filter_ratio"]}
        else:
            interpolate = interpolate_pose
            title = "request_servop"

            def payload(values):
                return {"left_pos": values[:7], "right_pos": values[7:]}

        ramp = interpolate(start, target, duration_sec=duration, rate_hz=rate)
        total = duration + HOLD_SEC
        command.drain_notifications()
        start_time = time.monotonic() + 0.05
        ramp_sends = []
        with ThreadPoolExecutor(max_workers=1) as pool:
            recording = pool.submit(observe, start_time, total)
            for sample in ramp:
                sleep_until(start_time + sample.t)
                ramp_sends.append(time.monotonic() - start_time)
                command.request(title, payload(sample.values), expect_response=False)
            for index in range(1, round(HOLD_SEC * rate) + 1):
                sleep_until(start_time + duration + index / rate)
                command.request(title, payload(target), expect_response=False)
            samples = recording.result()

        notifications = [item for item in command.drain_notifications()
                         if item.get("title", "").lower() in {"notify_servoj", "notify_servop"}]
        if notifications:
            raise RuntimeError(f"{name} emitted failure notifications: {notifications}")
        result = {"name": name, "duration_sec": duration, "hold_sec": HOLD_SEC, "rate_hz": rate,
                  "start": start, "target": target, "ramp_send_t_sec": ramp_sends, "samples": samples}
        result["tracking"] = summarize_track(result, samples, start, target)
        return result

    command.connect()
    try:
        observer.connect()
        head = cases["H_SAFE"]["target_head"]
        start_q = cases["J_PREP"]["target_q"] + head
        goal_q = cases["SJ_TRACK"]["target_q"] + cases["SJ_TRACK"]["target_head"]
        move("request_moveh", {"joint": head, "time": 1.0})
        move("request_movej", {"joint": start_q[:14], "time": 2.0})
        stable = wait_for_joint_target(
            command, start_q, indices=range(16), tolerance_rad=0.004,
            velocity_rad_s=0.04, timeout_sec=5.0, poll_sec=0.05, stable_samples=4,
        )
        if not stable.ok:
            raise RuntimeError(f"ServoJ start was not reached: {stable.code} ({stable.max_error:.6f} rad)")
        servo_j = track("servoJ", cases["SJ_TRACK"], start_q, goal_q)

        start_pose = cases["P_REACHABLE"]["target_pose"]
        move("request_movep", {"pos": start_pose, "time": 2.0})
        stable = wait_for_pose_target(
            command, start_pose, position_tolerance_m=0.002, orientation_tolerance_rad=0.01,
            timeout_sec=6.0, poll_sec=0.05, stable_samples=5,
        )
        if not stable.ok:
            raise RuntimeError(f"ServoP start was not reached: {stable.code} (error {stable.max_error:.6f})")
        servo_p = track("servoP", cases["SP_TRACK"], start_pose, cases["SP_TRACK"]["target_pose"])
        return {"backend": backend, "sample_hz": SAMPLE_HZ, "cases": [servo_j, servo_p]}
    finally:
        observer.close()
        command.close()


def compare(first, second):
    results = []
    for left, right in zip(first["cases"], second["cases"], strict=True):
        fields = ("name", "start", "target", "rate_hz", "duration_sec", "hold_sec")
        if any(left[field] != right[field] for field in fields):
            raise ValueError("The recordings do not use the same reference inputs")
        matching = {row["slot"]: row for row in right["samples"]}
        streams = {"q_rad": [], "dq_rad_s": [], "tau_nm": [], "position_mm": []}
        for row in left["samples"]:
            other = matching.get(row["slot"])
            if other is None:
                continue
            if abs(row["joint_t_sec"] - other["joint_t_sec"]) <= PAIR_TOLERANCE_SEC:
                for field, label in (("q", "q_rad"), ("dq", "dq_rad_s"), ("tau", "tau_nm")):
                    streams[label].append(joint_distance(row[field], other[field]))
            if abs(row["pose_t_sec"] - other["pose_t_sec"]) <= PAIR_TOLERANCE_SEC:
                streams["position_mm"].append(position_distance(row["pose"], other["pose"]))
        results.append({
            "case": left["name"],
            "first_tracking": left["tracking"], "second_tracking": right["tracking"],
            "between_backends": {key: {"samples": len(values), **summary(values)} for key, values in streams.items()},
        })
    return {"first": first["backend"], "second": second["backend"], "cases": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    recorder = modes.add_parser("capture")
    recorder.add_argument("backend", choices=("mujoco", "isaac"))
    recorder.add_argument("output", type=Path)
    reducer = modes.add_parser("compare")
    reducer.add_argument("first", type=Path)
    reducer.add_argument("second", type=Path)
    args = parser.parse_args()
    if args.mode == "capture":
        result = capture(args.backend)
        args.output.write_text(json.dumps(result, allow_nan=False) + "\n")
        result = {case["name"]: case["tracking"] for case in result["cases"]}
    else:
        result = compare(*(json.loads(path.read_text()) for path in (args.first, args.second)))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
