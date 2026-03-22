#!/usr/bin/env python3

"""
tf_rate_check.py

Measure dynamic TF update rate for TurtleBot3.

Checks:
- odom -> base_footprint   (preferred)
- odom -> base_link        (fallback)
- map -> odom              (optional, if available)

Method:
- sample transforms repeatedly
- track header timestamp changes
- compute effective update rate from unique TF timestamps

Also reports:
- per-second progress while recording
- average sampler rate of this checker node

Writes one summary row to the shared TF validation CSV.
"""

import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from tb3_tf_validation.result_utils import append_result


class TFRateCheck(Node):
    def __init__(self) -> None:
        super().__init__('tf_rate_check')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sample_period = 0.05   # target: 20 Hz sampling
        self.test_duration = 5.0    # seconds

        self.primary_candidates: List[Tuple[str, str]] = [
            ('odom', 'base_footprint'),
            ('odom', 'base_link'),
        ]

        self.selected_pairs: List[Tuple[str, str]] = []
        self.samples: Dict[Tuple[str, str], List[float]] = {}

        self.started = False
        self.elapsed_time = 0.0

        self.total_sample_cycles = 0
        self.wall_start_time: Optional[float] = None
        self.last_progress_second = 0

        self.start_timer = self.create_timer(2.0, self.start_check)
        self.sample_timer = None

    def frame_exists(self, frame_name: str) -> bool:
        try:
            yaml_text = self.tf_buffer.all_frames_as_yaml()
            return frame_name in yaml_text
        except Exception:
            return False

    def get_transform(self, parent: str, child: str) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        except Exception as exc:
            self.get_logger().warn(
                f'Unexpected error getting transform {parent} -> {child}: {exc}'
            )
            return None

    def stamp_to_sec(self, tf_msg: TransformStamped) -> float:
        return float(tf_msg.header.stamp.sec) + float(tf_msg.header.stamp.nanosec) * 1e-9

    def transform_exists(self, parent: str, child: str) -> bool:
        return self.get_transform(parent, child) is not None

    def start_check(self) -> None:
        if self.started:
            return
        self.started = True

        self.get_logger().info('========== TF RATE CHECK ==========')

        chosen_primary = None
        for parent, child in self.primary_candidates:
            if self.transform_exists(parent, child):
                chosen_primary = (parent, child)
                break

        if chosen_primary is None:
            self.get_logger().error(
                '[FAIL] Could not resolve odom -> base_footprint or odom -> base_link'
            )
            append_result(
                'tf_rate_check',
                'FAIL',
                'no dynamic TF pair resolved',
                'could not resolve odom->base_footprint or odom->base_link'
            )
            self.shutdown()
            return

        self.selected_pairs.append(chosen_primary)
        self.samples[chosen_primary] = []
        self.get_logger().info(
            f'[INFO] Primary dynamic TF selected: {chosen_primary[0]} -> {chosen_primary[1]}'
        )

        if self.frame_exists('map') and self.transform_exists('map', 'odom'):
            map_pair = ('map', 'odom')
            self.selected_pairs.append(map_pair)
            self.samples[map_pair] = []
            self.get_logger().info('[INFO] Optional dynamic TF selected: map -> odom')
        else:
            self.get_logger().info('[INFO] map -> odom not available, skipping')

        self.wall_start_time = time.time()
        self.last_progress_second = 0

        self.get_logger().info(
            f'[INFO] Sampling for {self.test_duration:.1f} sec at target {1.0 / self.sample_period:.1f} Hz'
        )

        self.destroy_timer(self.start_timer)
        self.sample_timer = self.create_timer(self.sample_period, self.collect_samples)

    def collect_samples(self) -> None:
        self.elapsed_time += self.sample_period
        self.total_sample_cycles += 1

        for pair in self.selected_pairs:
            parent, child = pair
            tf_msg = self.get_transform(parent, child)

            if tf_msg is None:
                self.get_logger().warn(f'[WARN] Sample lookup failed: {parent} -> {child}')
                continue

            stamp_sec = self.stamp_to_sec(tf_msg)
            self.samples[pair].append(stamp_sec)

        current_second = int(self.elapsed_time)
        if current_second > self.last_progress_second:
            self.last_progress_second = current_second

            wall_elapsed = max(1e-9, time.time() - self.wall_start_time) if self.wall_start_time else 1e-9
            avg_sampler_rate = self.total_sample_cycles / wall_elapsed

            self.get_logger().info(
                f'[INFO] Progress: {min(self.elapsed_time, self.test_duration):.1f}/{self.test_duration:.1f} sec '
                f'| sample cycles: {self.total_sample_cycles} '
                f'| avg sampler rate: {avg_sampler_rate:.2f} Hz'
            )

            for parent, child in self.selected_pairs:
                count = len(self.samples[(parent, child)])
                self.get_logger().info(
                    f'[INFO]   {parent} -> {child}: {count} timestamp samples recorded'
                )

        if self.elapsed_time >= self.test_duration:
            self.analyze_samples()

    def analyze_samples(self) -> None:
        if self.sample_timer is not None:
            self.destroy_timer(self.sample_timer)

        self.get_logger().info('')
        self.get_logger().info('========== ANALYSIS ==========')

        overall_pass = True
        notes_parts = []
        measurement_parts = []

        wall_elapsed = max(1e-9, time.time() - self.wall_start_time) if self.wall_start_time else 1e-9
        avg_sampler_rate = self.total_sample_cycles / wall_elapsed

        self.get_logger().info(f'[INFO] Total wall time: {wall_elapsed:.3f} sec')
        self.get_logger().info(f'[INFO] Total sample cycles: {self.total_sample_cycles}')
        self.get_logger().info(f'[INFO] Average sampler rate: {avg_sampler_rate:.2f} Hz')

        for pair in self.selected_pairs:
            parent, child = pair
            stamps = self.samples[pair]

            if len(stamps) < 2:
                self.get_logger().error(f'[FAIL] Not enough samples for {parent} -> {child}')
                overall_pass = False
                notes_parts.append(f'not enough samples for {parent}->{child}')
                continue

            unique_stamps = []
            for s in stamps:
                if not unique_stamps or abs(s - unique_stamps[-1]) > 1e-9:
                    unique_stamps.append(s)

            num_total = len(stamps)
            num_unique = len(unique_stamps)
            num_updates = max(0, num_unique - 1)

            if num_unique >= 2:
                time_span = unique_stamps[-1] - unique_stamps[0]
            else:
                time_span = 0.0

            rate_hz = (num_updates / time_span) if time_span > 0.0 else 0.0
            repeated_count = num_total - num_unique

            self.get_logger().info(f'[INFO] Transform checked: {parent} -> {child}')
            self.get_logger().info(f'[INFO] Total samples: {num_total}')
            self.get_logger().info(f'[INFO] Unique TF timestamps: {num_unique}')
            self.get_logger().info(f'[INFO] Repeated timestamps: {repeated_count}')
            self.get_logger().info(f'[INFO] TF timestamp span: {time_span:.3f} sec')
            self.get_logger().info(f'[INFO] Estimated TF update rate: {rate_hz:.2f} Hz')

            measurement_parts.append(f'{parent}->{child}: {rate_hz:.2f} Hz')

            pair_pass = True

            if num_unique < 2:
                self.get_logger().error(f'[FAIL] TF did not appear to update: {parent} -> {child}')
                pair_pass = False
                notes_parts.append(f'{parent}->{child} did not update')
            else:
                self.get_logger().info(f'[PASS] TF timestamps updated: {parent} -> {child}')

            if rate_hz >= 5.0:
                self.get_logger().info(f'[PASS] TF update rate is reasonable: {rate_hz:.2f} Hz')
            else:
                self.get_logger().error(f'[FAIL] TF update rate is too low: {rate_hz:.2f} Hz')
                pair_pass = False
                notes_parts.append(f'{parent}->{child} low rate {rate_hz:.2f} Hz')

            if not pair_pass:
                overall_pass = False

            self.get_logger().info('')

        self.get_logger().info('========== SUMMARY ==========')

        if overall_pass:
            self.get_logger().info('[PASS] TF rate check passed')
            status = 'PASS'
        else:
            self.get_logger().error('[FAIL] TF rate check failed')
            status = 'FAIL'

        measurement = ' | '.join(measurement_parts) if measurement_parts else 'no rate computed'

        if not notes_parts:
            notes = f'avg sampler rate={avg_sampler_rate:.2f} Hz'
        else:
            notes = '; '.join(notes_parts + [f'avg sampler rate={avg_sampler_rate:.2f} Hz'])

        append_result(
            'tf_rate_check',
            status,
            measurement,
            notes
        )

        self.shutdown()

    def shutdown(self) -> None:
        self.get_logger().info('TF rate check complete. Shutting down.')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TFRateCheck()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()