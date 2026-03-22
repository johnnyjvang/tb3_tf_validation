#!/usr/bin/env python3

"""
tf_lookup_test.py

Repeatedly test TF lookup reliability for TurtleBot3.

Checks repeated lookup success/failure for:
- odom -> base_footprint   (preferred)
- odom -> base_link        (fallback)
- map -> odom              (optional)
- base_link -> imu_link    (optional if present)
- base_link -> laser/base_scan (optional if present)

Reports:
- per-second progress
- average sampler rate
- success/failure counts
- success percentage for each transform

Writes one summary row to the shared TF validation CSV.
"""

import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from tb3_tf_validation.result_utils import append_result


class TFLookupTest(Node):
    def __init__(self) -> None:
        super().__init__('tf_lookup_test')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sample_period = 0.05
        self.test_duration = 5.0

        self.dynamic_candidates: List[Tuple[str, str]] = [
            ('odom', 'base_footprint'),
            ('odom', 'base_link'),
        ]
        self.lidar_candidates = ['laser', 'base_scan']

        self.selected_pairs: List[Tuple[str, str]] = []

        self.success_counts: Dict[Tuple[str, str], int] = {}
        self.failure_counts: Dict[Tuple[str, str], int] = {}

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

    def transform_exists(self, parent: str, child: str) -> bool:
        try:
            self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False
        except Exception as exc:
            self.get_logger().warn(
                f'Unexpected error checking transform {parent} -> {child}: {exc}'
            )
            return False

    def find_lidar_frame(self) -> Optional[str]:
        for frame in self.lidar_candidates:
            if self.frame_exists(frame):
                return frame
        return None

    def add_pair(self, parent: str, child: str, label: str) -> None:
        pair = (parent, child)
        self.selected_pairs.append(pair)
        self.success_counts[pair] = 0
        self.failure_counts[pair] = 0
        self.get_logger().info(f'[INFO] Added {label}: {parent} -> {child}')

    def start_check(self) -> None:
        if self.started:
            return
        self.started = True

        self.get_logger().info('========== TF LOOKUP TEST ==========')

        chosen_dynamic = None
        for parent, child in self.dynamic_candidates:
            if self.transform_exists(parent, child):
                chosen_dynamic = (parent, child)
                break

        if chosen_dynamic is None:
            self.get_logger().error(
                '[FAIL] Could not resolve odom -> base_footprint or odom -> base_link'
            )
            append_result(
                'tf_lookup_test',
                'FAIL',
                'no dynamic TF pair resolved',
                'could not resolve odom->base_footprint or odom->base_link'
            )
            self.shutdown()
            return

        self.add_pair(chosen_dynamic[0], chosen_dynamic[1], 'primary dynamic TF')

        if self.frame_exists('map') and self.transform_exists('map', 'odom'):
            self.add_pair('map', 'odom', 'optional dynamic TF')
        else:
            self.get_logger().info('[INFO] map -> odom not available, skipping')

        if self.frame_exists('imu_link'):
            self.add_pair('base_link', 'imu_link', 'IMU TF')
        else:
            self.get_logger().info('[INFO] imu_link not available, skipping')

        lidar_frame = self.find_lidar_frame()
        if lidar_frame is not None:
            self.add_pair('base_link', lidar_frame, 'lidar TF')
        else:
            self.get_logger().info('[INFO] lidar frame not available, skipping')

        if not self.selected_pairs:
            self.get_logger().error('[FAIL] No transform pairs selected for lookup test')
            append_result(
                'tf_lookup_test',
                'FAIL',
                'no transform pairs selected',
                'internal selection failure'
            )
            self.shutdown()
            return

        self.wall_start_time = time.time()
        self.last_progress_second = 0

        self.get_logger().info(
            f'[INFO] Running repeated lookups for {self.test_duration:.1f} sec '
            f'at target {1.0 / self.sample_period:.1f} Hz'
        )

        self.destroy_timer(self.start_timer)
        self.sample_timer = self.create_timer(self.sample_period, self.collect_samples)

    def collect_samples(self) -> None:
        self.elapsed_time += self.sample_period
        self.total_sample_cycles += 1

        for pair in self.selected_pairs:
            parent, child = pair
            if self.transform_exists(parent, child):
                self.success_counts[pair] += 1
            else:
                self.failure_counts[pair] += 1

        current_second = int(self.elapsed_time)
        if current_second > self.last_progress_second:
            self.last_progress_second = current_second

            wall_elapsed = (
                max(1e-9, time.time() - self.wall_start_time)
                if self.wall_start_time else 1e-9
            )
            avg_sampler_rate = self.total_sample_cycles / wall_elapsed

            self.get_logger().info(
                f'[INFO] Progress: {min(self.elapsed_time, self.test_duration):.1f}/'
                f'{self.test_duration:.1f} sec | sample cycles: {self.total_sample_cycles} '
                f'| avg sampler rate: {avg_sampler_rate:.2f} Hz'
            )

            for parent, child in self.selected_pairs:
                pair = (parent, child)
                s = self.success_counts[pair]
                f = self.failure_counts[pair]
                total = s + f
                pct = (100.0 * s / total) if total > 0 else 0.0
                self.get_logger().info(
                    f'[INFO]   {parent} -> {child}: successes={s}, failures={f}, '
                    f'success rate={pct:.1f}%'
                )

        if self.elapsed_time >= self.test_duration:
            self.analyze_results()

    def analyze_results(self) -> None:
        if self.sample_timer is not None:
            self.destroy_timer(self.sample_timer)

        self.get_logger().info('')
        self.get_logger().info('========== ANALYSIS ==========')

        overall_pass = True
        notes_parts = []
        measurement_parts = []

        wall_elapsed = (
            max(1e-9, time.time() - self.wall_start_time)
            if self.wall_start_time else 1e-9
        )
        avg_sampler_rate = self.total_sample_cycles / wall_elapsed

        self.get_logger().info(f'[INFO] Total wall time: {wall_elapsed:.3f} sec')
        self.get_logger().info(f'[INFO] Total sample cycles: {self.total_sample_cycles}')
        self.get_logger().info(f'[INFO] Average sampler rate: {avg_sampler_rate:.2f} Hz')

        for parent, child in self.selected_pairs:
            pair = (parent, child)
            successes = self.success_counts[pair]
            failures = self.failure_counts[pair]
            total = successes + failures
            success_rate = (100.0 * successes / total) if total > 0 else 0.0

            self.get_logger().info('')
            self.get_logger().info(f'[INFO] Transform checked: {parent} -> {child}')
            self.get_logger().info(f'[INFO] Total lookups: {total}')
            self.get_logger().info(f'[INFO] Successful lookups: {successes}')
            self.get_logger().info(f'[INFO] Failed lookups: {failures}')
            self.get_logger().info(f'[INFO] Success rate: {success_rate:.2f}%')

            measurement_parts.append(f'{parent}->{child}: {success_rate:.1f}%')

            pair_pass = True

            if total == 0:
                self.get_logger().error(f'[FAIL] No lookups performed for {parent} -> {child}')
                pair_pass = False
                notes_parts.append(f'no lookups for {parent}->{child}')
            elif success_rate >= 95.0:
                self.get_logger().info(
                    f'[PASS] Lookup reliability is good for {parent} -> {child}'
                )
            else:
                self.get_logger().error(
                    f'[FAIL] Lookup reliability is too low for {parent} -> {child}'
                )
                pair_pass = False
                notes_parts.append(f'{parent}->{child} low success rate {success_rate:.1f}%')

            if failures == 0:
                self.get_logger().info(
                    f'[PASS] No lookup failures observed for {parent} -> {child}'
                )
            else:
                self.get_logger().warn(
                    f'[WARN] Lookup failures observed for {parent} -> {child}: {failures}'
                )

            if not pair_pass:
                overall_pass = False

        self.get_logger().info('')
        self.get_logger().info('========== SUMMARY ==========')

        if overall_pass:
            self.get_logger().info('[PASS] TF lookup test passed')
            status = 'PASS'
        else:
            self.get_logger().error('[FAIL] TF lookup test failed')
            status = 'FAIL'

        measurement = ' | '.join(measurement_parts) if measurement_parts else 'no lookup stats computed'

        if not notes_parts:
            notes = f'avg sampler rate={avg_sampler_rate:.2f} Hz'
        else:
            notes = '; '.join(notes_parts + [f'avg sampler rate={avg_sampler_rate:.2f} Hz'])

        append_result(
            'tf_lookup_test',
            status,
            measurement,
            notes
        )

        self.shutdown()

    def shutdown(self) -> None:
        self.get_logger().info('TF lookup test complete. Shutting down.')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TFLookupTest()

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