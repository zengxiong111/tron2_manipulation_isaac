import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mros_timing_observer import StreamStats, percentile, source_age_measure


class MrosTimingReducerTest(unittest.TestCase):
    def test_percentiles_interpolate_and_handle_empty_input(self):
        self.assertEqual(percentile([0.0, 10.0], 25), 2.5)
        self.assertIsNone(percentile([], 50))

    def test_delivery_rate_and_interarrival_percentiles(self):
        stats = StreamStats(expected_rate_hz=100.0, stale_after_ms=20.0)
        shape = {"na": 16, "names_count": 0, "array_lengths": {"q": 16}}
        for arrival in (0, 10_000_000, 20_000_000, 30_000_000):
            stats.add(arrival, 100_000_000_000 + arrival, None, shape)

        result = stats.summary(window_seconds=0.04)

        self.assertEqual(result["message_count"], 4)
        self.assertEqual(result["delivered_rate_hz"], 100.0)
        self.assertEqual(result["sample_span_rate_hz"], 100.0)
        self.assertEqual(result["interarrival_ms"]["p50"], 10.0)
        self.assertEqual(result["confirmed_lost_messages"], None)

    def test_long_gap_is_estimated_not_reported_as_confirmed_loss(self):
        stats = StreamStats(expected_rate_hz=300.0, stale_after_ms=20.0)
        shape = {"na": 16, "names_count": 0, "array_lengths": {"q": 16}}
        for arrival in (0, 3_333_333, 6_666_666, 16_666_666):
            stats.add(arrival, 100_000_000_000, None, shape)

        result = stats.summary(window_seconds=0.02)

        self.assertEqual(result["arrival_gaps_over_2x_expected"], 1)
        self.assertEqual(result["estimated_missing_update_ticks"], 2)
        self.assertIsNone(result["confirmed_lost_messages"])

    def test_source_age_requires_a_plausible_shared_monotonic_clock(self):
        receive_monotonic_ns = 1_000_000_000_000
        receive_wall_ns = 1_700_000_000_000_000_000
        self.assertEqual(
            source_age_measure(
                receive_monotonic_ns - 5_000_000,
                receive_monotonic_ns,
                receive_wall_ns,
            ),
            (5.0, "monotonic"),
        )
        self.assertEqual(source_age_measure(None, receive_monotonic_ns, receive_wall_ns), (None, None))
        self.assertEqual(source_age_measure(1, receive_monotonic_ns, receive_wall_ns), (None, None))

    def test_wall_clock_source_age_is_selected_when_monotonic_is_incompatible(self):
        receive_monotonic_ns = 1_000_000_000_000_000
        receive_wall_ns = 1_700_000_000_000_000_000

        age_ms, basis = source_age_measure(
            receive_wall_ns - 7_500_000,
            receive_monotonic_ns,
            receive_wall_ns,
        )

        self.assertAlmostEqual(age_ms, 7.5)
        self.assertEqual(basis, "wall_unix")

    def test_stale_age_count_uses_the_explicit_threshold(self):
        stats = StreamStats(expected_rate_hz=300.0, stale_after_ms=10.0)
        shape = {"na": 16, "names_count": 0, "array_lengths": {"q": 16}}
        wall = 100_000_000_000
        stats.add(wall, wall, wall - 5_000_000, shape)
        stats.add(wall + 1, wall + 1, wall - 25_000_000, shape)

        result = stats.summary(window_seconds=0.01)

        self.assertEqual(result["source_age_ms"]["eligible_samples"], 2)
        self.assertEqual(result["source_age_ms"]["over_threshold_count"], 1)
        self.assertAlmostEqual(result["source_age_ms"]["p50"], 15.0, places=5)


if __name__ == "__main__":
    unittest.main()
