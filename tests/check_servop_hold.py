"""Live 0.1 mm steady-state check for SP_TRACK inside scripts/run_sim.sh."""

import json
import os
from pathlib import Path
import sys
import time

import yaml


HOLD_SECONDS = 3.0
MAX_STEADY_ERROR_M = 0.0001


def main() -> int:
    upstream = Path(os.environ["TRON2_UPSTREAM"]).resolve() / "Tron2_arm_tests"
    sys.path.insert(0, str(upstream))
    from dual_arm_protocol_tests.client import DualArmProtocolClient
    from dual_arm_protocol_tests.metrics import pose_error
    from dual_arm_protocol_tests.motion import wait_for_joint_target
    from dual_arm_protocol_tests.trajectory import interpolate_pose

    cases_doc = yaml.safe_load((upstream / "config/test_cases.yaml").read_text())
    cases = {case["id"]: case for case in cases_doc["cases"]}
    start_q = cases["SJ_TRACK"]["target_q"]
    case = cases["SP_TRACK"]
    target = case["target_pose"]
    rate = case["send_rate_hz"]
    client = DualArmProtocolClient("127.0.0.1", accid="DACH_TRON2A_001")

    def send_pose(pose, deadline):
        time.sleep(max(0.0, deadline - time.monotonic()))
        client.request(
            "request_servop", {"left_pos": pose[:7], "right_pos": pose[7:]},
            expect_response=False,
        )

    client.connect()
    try:
        reply = client.request("request_movej", {"joint": start_q, "time": 1.2})
        if reply["data"]["result"] != "success":
            raise RuntimeError(f"MoveJ preparation failed: {reply}")
        stable = wait_for_joint_target(
            client, start_q, indices=range(14), tolerance_rad=0.004,
            velocity_rad_s=0.04, timeout_sec=5.0, poll_sec=0.05, stable_samples=4,
        )
        if not stable.ok:
            raise RuntimeError(f"Robot did not reach the ServoP starting pose: {stable.code}")

        start_pose = client.get_move_pose().as_flat_pose()
        samples = interpolate_pose(
            start_pose, target, duration_sec=case["duration"], rate_hz=rate,
        )
        start = time.monotonic()
        for sample in samples:
            send_pose(sample.values, start + sample.t)
        end_of_ramp = time.monotonic()
        arrival = pose_error(target, client.get_move_pose().as_flat_pose()).max_position
        for index in range(1, round(HOLD_SECONDS * rate) + 1):
            send_pose(target, end_of_ramp + index / rate)
        steady = pose_error(target, client.get_move_pose().as_flat_pose()).max_position
        passed = steady <= MAX_STEADY_ERROR_M
        print(json.dumps({
            "case": case["id"], "rate_hz": rate, "hold_s": HOLD_SECONDS,
            "arrival_error_mm": arrival * 1000, "steady_error_mm": steady * 1000,
            "max_steady_error_mm": MAX_STEADY_ERROR_M * 1000, "passed": passed,
        }), flush=True)
        return 0 if passed else 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
