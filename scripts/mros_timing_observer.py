#!/usr/bin/env python3
"""Capture read-only MROS motor topic timing as JSON Lines.

The observer records ROS 2 callback arrival time after the MROS bridge. It
does not publish or command robot topics. MROS messages have no sequence
counter, so packet loss cannot be counted exactly; long arrival gaps are
reported as estimates against the expected update rate.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time
from typing import Any


MOTOR_COUNT = 16
SOURCE_CLOCK_TOLERANCE_NS = 60 * 1_000_000_000


def percentile(values: list[float], p: float) -> float | None:
    """Linear-interpolated percentile, with p in [0, 100]."""

    if not values:
        return None
    ordered = sorted(values)
    rank = (len(ordered) - 1) * p / 100.0
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def header_stamp_ns(message: Any) -> int | None:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else None


def source_age_measure(
    stamp_ns: int | None,
    receive_monotonic_ns: int,
    receive_wall_ns: int,
) -> tuple[float | None, str | None]:
    """Match a source timestamp to monotonic or wall time only when plausible."""
    if stamp_ns is None:
        return None, None
    age_ns = receive_monotonic_ns - stamp_ns
    if abs(age_ns) <= SOURCE_CLOCK_TOLERANCE_NS:
        return age_ns / 1_000_000.0, "monotonic"
    age_ns = receive_wall_ns - stamp_ns
    if abs(age_ns) <= SOURCE_CLOCK_TOLERANCE_NS:
        return age_ns / 1_000_000.0, "wall_unix"
    return None, None


def message_shape(topic: str, message: Any) -> dict[str, Any]:
    fields = ("q", "v", "tau")
    shape: dict[str, Any] = {
        "na": int(getattr(message, "na", 0)),
        "names_count": len(getattr(message, "names", [])),
        "array_lengths": {field: len(getattr(message, field, [])) for field in fields},
    }
    if topic == "/motor/cmd":
        shape["array_lengths"].update({
            "kp": len(getattr(message, "kp", [])),
            "kd": len(getattr(message, "kd", [])),
        })
    return shape


class StreamStats:
    """Bounded-cost timing reducer for one subscribed topic."""

    def __init__(self, expected_rate_hz: float, stale_after_ms: float) -> None:
        self.expected_rate_hz = expected_rate_hz
        self.stale_after_ms = stale_after_ms
        self.arrival_ns: list[int] = []
        self.source_age_ms_values: list[float] = []
        self.source_stamp_ns: list[int] = []
        self.shape: dict[str, Any] | None = None
        self.stale_count = 0
        self.incompatible_stamp_count = 0
        self.source_age_clock_counts: dict[str, int] = {}

    def add(
        self,
        arrival_ns: int,
        wall_ns: int,
        stamp_ns: int | None,
        shape: dict[str, Any],
    ) -> dict[str, Any]:
        age_ms, age_clock = source_age_measure(stamp_ns, arrival_ns, wall_ns)
        self.arrival_ns.append(arrival_ns)
        if stamp_ns is not None:
            self.source_stamp_ns.append(stamp_ns)
            if age_ms is None:
                self.incompatible_stamp_count += 1
            else:
                self.source_age_ms_values.append(age_ms)
                assert age_clock is not None
                self.source_age_clock_counts[age_clock] = self.source_age_clock_counts.get(age_clock, 0) + 1
                if age_ms > self.stale_after_ms:
                    self.stale_count += 1
        self.shape = shape
        return {
            "arrival_monotonic_ns": arrival_ns,
            "wall_unix_ns": wall_ns,
            "header_stamp_ns": stamp_ns,
            "source_age_ms": age_ms,
            "source_age_clock_basis": age_clock,
            **shape,
        }

    def summary(self, window_seconds: float) -> dict[str, Any]:
        intervals_ms = [
            (after - before) / 1_000_000.0
            for before, after in zip(self.arrival_ns, self.arrival_ns[1:])
        ]
        period_ms = 1000.0 / self.expected_rate_hz
        extended_gaps = [interval for interval in intervals_ms if interval > 2.0 * period_ms]
        estimated_missing_ticks = sum(
            max(0, round(interval / period_ms) - 1)
            for interval in extended_gaps
        )
        sample_span_s = (
            (self.arrival_ns[-1] - self.arrival_ns[0]) / 1_000_000_000.0
            if len(self.arrival_ns) > 1
            else 0.0
        )
        count = len(self.arrival_ns)
        return {
            "message_count": count,
            "capture_window_s": window_seconds,
            "delivered_rate_hz": count / window_seconds if window_seconds > 0 else None,
            "sample_span_rate_hz": (count - 1) / sample_span_s if sample_span_s > 0 else None,
            "expected_rate_hz": self.expected_rate_hz,
            "interarrival_ms": {
                "mean": statistics.fmean(intervals_ms) if intervals_ms else None,
                "p50": percentile(intervals_ms, 50),
                "p95": percentile(intervals_ms, 95),
                "p99": percentile(intervals_ms, 99),
                "max": max(intervals_ms) if intervals_ms else None,
            },
            "arrival_gaps_over_2x_expected": len(extended_gaps),
            "estimated_missing_update_ticks": estimated_missing_ticks,
            "confirmed_lost_messages": None,
            "loss_limit": "No sequence counter exists in controller_msgs JointCmd/JointState; gap counts are estimates, not confirmed drops.",
            "source_stamp_samples": len(self.source_stamp_ns),
            "source_stamp_clock_incompatible": self.incompatible_stamp_count,
            "source_age_ms": {
                "eligible_samples": len(self.source_age_ms_values),
                "stale_after_ms": self.stale_after_ms,
                "over_threshold_count": self.stale_count,
                "p50": percentile(self.source_age_ms_values, 50),
                "p95": percentile(self.source_age_ms_values, 95),
                "p99": percentile(self.source_age_ms_values, 99),
                "max": max(self.source_age_ms_values) if self.source_age_ms_values else None,
                "clock_basis_counts": self.source_age_clock_counts,
                "clock_basis": (
                    next(iter(self.source_age_clock_counts))
                    if len(self.source_age_clock_counts) == 1
                    else "per-sample; see clock_basis_counts"
                    if self.source_age_clock_counts
                    else "unavailable; no plausible same-host clock"
                ),
            },
            "message_shape": self.shape,
        }


class TimingObserver:
    """Read-only callbacks; ROS imports are deferred for reducer unit tests."""

    def __init__(self, node: Any, output, engine: str, expected_rate_hz: float, stale_after_ms: float, observer_boundary: str) -> None:
        self.node = node
        self.output = output
        self.engine = engine
        self.expected_rate_hz = expected_rate_hz
        self.stale_after_ms = stale_after_ms
        self.observer_boundary = observer_boundary
        self.ready_topics: set[str] = set()
        self.active = False
        self.streams = {
            "/motor/cmd": StreamStats(expected_rate_hz, stale_after_ms),
            "/motor/state": StreamStats(expected_rate_hz, stale_after_ms),
        }
        self.capture_start_ns: int | None = None
        self.capture_start_wall_ns: int | None = None

    def on_message(self, topic: str, message: Any) -> None:
        self.ready_topics.add(topic)
        if not self.active:
            return
        arrival_ns = time.monotonic_ns()
        wall_ns = time.time_ns()
        stamp_ns = header_stamp_ns(message)
        event = self.streams[topic].add(
            arrival_ns, wall_ns, stamp_ns, message_shape(topic, message),
        )
        self.output.write(json.dumps({"record": "sample", "engine": self.engine, "topic": topic, **event}, separators=(",", ":")) + "\n")

    def begin(self, ready_file: Path | None) -> None:
        self.capture_start_ns = time.monotonic_ns()
        self.capture_start_wall_ns = time.time_ns()
        self.active = True
        self.output.write(json.dumps({
            "record": "metadata",
            "engine": self.engine,
            "capture_start_monotonic_ns": self.capture_start_ns,
            "capture_start_wall_unix_ns": self.capture_start_wall_ns,
            "observer_boundary": self.observer_boundary,
            "observer_direction": "Read-only ROS 2 subscriptions; observer never republishes motor data.",
            "controller_port_5000": "not used by this observer",
            "expected_rate_hz": self.expected_rate_hz,
            "stale_after_ms": self.stale_after_ms,
            "loss_observability": "MROS messages have no sequence counter; estimated interval gaps only.",
            "source_age_basis": "Each Header stamp is compared with same-host monotonic_ns and wall time; the matching plausible clock is recorded per sample; implausible offsets are null.",
        }, separators=(",", ":")) + "\n")
        self.output.flush()
        if ready_file is not None:
            try:
                with ready_file.open("x") as marker:
                    marker.write("ready\n")
            except FileExistsError:
                raise RuntimeError(f"refusing to overwrite ready marker: {ready_file}") from None
        print("TIMING_OBSERVER_READY", flush=True)

    def finish(self, duration_s: float) -> None:
        assert self.capture_start_ns is not None
        actual_window_s = (time.monotonic_ns() - self.capture_start_ns) / 1_000_000_000.0
        for topic, stats in self.streams.items():
            self.output.write(json.dumps({
                "record": "summary",
                "engine": self.engine,
                "topic": topic,
                "requested_window_s": duration_s,
                **stats.summary(actual_window_s),
            }, separators=(",", ":")) + "\n")
        self.output.flush()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", required=True, choices=("mujoco", "isaac"))
    parser.add_argument("--output", required=True, type=Path, help="JSONL path outside the repository")
    parser.add_argument("--seconds", type=float, default=30.0, help="capture duration after both topics are seen")
    parser.add_argument("--expected-rate-hz", type=float, default=300.0)
    parser.add_argument("--stale-after-ms", type=float, default=20.0)
    parser.add_argument("--startup-timeout-s", type=float, default=30.0)
    parser.add_argument("--ready-file", type=Path, help="write this file when capture begins")
    parser.add_argument(
        "--observer-boundary",
        default="MROS DDS topic -> observer ros2-bridger -> ROS 2 subscription callback",
        help="describe where callback arrivals are measured",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    for name in ("seconds", "expected_rate_hz", "stale_after_ms", "startup_timeout_s"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise SystemExit(f"--{name.replace('_', '-')} must be finite and positive")
    output_path = args.output.expanduser().resolve()
    repo_root = Path(__file__).resolve().parents[1]
    if output_path == repo_root or repo_root in output_path.parents:
        raise SystemExit("timing output must be outside the repository")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        raise SystemExit(f"refusing to overwrite timing output: {output_path}")
    if args.ready_file is not None:
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)

    import rclpy
    from controller_msgs.msg import JointCmd, JointState
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("mros_timing_observer")
    try:
        output = output_path.open("x", buffering=64 * 1024)
    except FileExistsError:
        raise SystemExit(f"refusing to overwrite timing output: {output_path}") from None
    observer = TimingObserver(
        node, output, args.engine, args.expected_rate_hz, args.stale_after_ms,
        args.observer_boundary,
    )
    qos = 10
    node.create_subscription(JointCmd, "/motor/cmd", lambda msg: observer.on_message("/motor/cmd", msg), qos)
    node.create_subscription(JointState, "/motor/state", lambda msg: observer.on_message("/motor/state", msg), qos)
    startup_deadline = time.monotonic() + args.startup_timeout_s
    try:
        while rclpy.ok() and not {"/motor/cmd", "/motor/state"}.issubset(observer.ready_topics):
            if time.monotonic() >= startup_deadline:
                raise TimeoutError(f"no command/state pair observed; saw {sorted(observer.ready_topics)}")
            rclpy.spin_once(node, timeout_sec=0.1)
        observer.begin(args.ready_file)
        deadline_ns = observer.capture_start_ns + int(args.seconds * 1_000_000_000)
        while rclpy.ok() and time.monotonic_ns() < deadline_ns:
            rclpy.spin_once(node, timeout_sec=0.01)
        observer.finish(args.seconds)
    finally:
        observer.output.close()
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"Timing capture saved to {output_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
