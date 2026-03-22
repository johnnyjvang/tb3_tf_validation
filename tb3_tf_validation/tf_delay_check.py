#!/usr/bin/env python3

"""
tf_delay_check.py

Measure TF timestamp delay / freshness for TurtleBot3.

Checks:
- odom -> base_footprint   (preferred)
- odom -> base_link        (fallback)
- map -> odom              (optional, if available)

Method:
- sample transforms repeatedly
- compare current ROS time to TF header timestamp
- compute delay statistics

Also reports:
- per-second progress while recording
- average sampler rate of this checker node
- warning if a clock mismatch is likely

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


class TFDelayCheck(Node):
    def __init__(self) -> None:
        super().__init__('tf_delay_check')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sample_period = 0.05
        self.test_duration = 5.0
        self.startup_wait_timeout = 10.0

        self.primary_candidates: List[Tuple[str, str]] = [
            ('odom', 'base_footprint'),
            ('odom', 'base_link'),
        ]

        self.selected_pairs: List[Tuple[str, str]] = []
        self.delay_samples: Dict[Tuple[str, str], List[float]] = {}

        self.state = 'startup_wait'
        self.state_wall_time = time.time()

        self.elapsed_time = 0.0
        self.total_sample_cycles = 0
        self.wall_start_time: Optional[float] = None
        self.last_progress_second = -1

        self.timer = self.create_timer(self.sample_period, self.run)

        self.get_logger().info('========== TF DELAY CHECK ==========')
        self.get_logger().info(f'[INFO] use_sim_time: {self.get_use_sim_time()}')

        if not self.get_use_sim_time():
            self.get_logger().warn(
                '[WARN] use_sim_time is false. If running in Gazebo, use: '
                '--ros-args -p use_sim_time:=true'
            )

    def get_use_sim_time(self) -> bool:
        try:
            return bool(self.get_parameter('use_sim_time').value)
        except Exception:
            return False

    def stamp_to_sec(self, sec: int, nanosec: int) -> float:
        return float(sec) + float(nanosec) * 1e-9

    def now_ros_time_sec(self) -> float:
        now_msg = self.get_clock().now().to_msg()
        return self.stamp_to_sec(now_msg.sec, now_msg.nanosec)

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

    def can_resolve_transform(self, parent: str, child: str) -> bool:
        return self.get_transform(parent, child) is not None

    def startup_wait_done(self) -> bool:
        ros_now = self.now_ros_time_sec()

        if self.get_use_sim_time() and ros_now <= 0.0:
            return False

        for parent, child in self.primary_candidates:
            if self.can_resolve_transform(parent, child):
                return True

        return False

    def choose_primary_pair(self) -> Optional[Tuple[str, str]]:
        for parent, child in self.primary_candidates:
            if self.can_resolve_transform(parent, child):
                return (parent, child)
        return None

    def begin_recording(self) -> None:
        chosen_primary = self.choose_primary_pair()
        if chosen_primary is None:
            self.get_logger().error(
                '[FAIL] Could not resolve odom -> base_footprint or odom -> base_link'
            )
            append_result(
                'tf_delay_check',
                'FAIL',
                'no dynamic TF pair resolved',
                'could not resolve odom->base_footprint or odom->base_link'
            )
            self.shutdown()
            return

        self.selected_pairs.append(chosen_primary)
        self.delay_samples[chosen_primary] = []
        self.get_logger().info(
            f'[INFO] Primary dynamic TF selected: {chosen_primary[0]} -> {chosen_primary[1]}'
        )

        if self.frame_exists('map') and self.can_resolve_transform('map', 'odom'):
            map_pair = ('map', 'odom')
            self.selected_pairs.append(map_pair)
            self.delay_samples[map_pair] = []
            self.get_logger().info('[INFO] Optional dynamic TF selected: map -> odom')
        else:
            self.get_logger().info('[INFO] map -> odom not available, skipping')

        self.state = 'recording'
        self.state_wall_time = time.time()
        self.wall_start_time = time.time()
        self.elapsed_time = 0.0
        self.total_sample_cycles = 0
        self.last_progress_second = -1

        self.get_logger().info(
            f'[INFO] Recording delay for {self.test_duration:.1f} sec at target '
            f'{1.0 / self.sample_period:.1f} Hz'
        )

    def run(self) -> None:
        if self.state == 'startup_wait':
            wait_elapsed = time.time() - self.state_wall_time
            ros_now = self.now_ros_time_sec()

            current_second = int(wait_elapsed)
            if current_second != self.last_progress_second:
                self.last_progress_second = current_second
                self.get_logger().info(
                    f'[INFO] Waiting for TF/clock... '
                    f'{wait_elapsed:.1f}/{self.startup_wait_timeout:.1f} sec '
                    f'| ros_time={ros_now:.3f}'
                )

            if self.startup_wait_done():
                self.get_logger().info('[PASS] TF and clock appear ready')
                self.begin_recording()
                return

            if wait_elapsed >= self.startup_wait_timeout:
                self.get_logger().error('[FAIL] Timed out waiting for TF/clock readiness.')

                notes = 'timed out waiting for TF/clock readiness'
                if self.get_use_sim_time() and ros_now <= 0.0:
                    self.get_logger().warn(
                        '[WARN] ROS time never advanced above zero. Check that /clock is active.'
                    )
                    notes += '; ROS time never advanced above zero'
                else:
                    self.get_logger().warn(
                        '[WARN] Clock may be running, but TF was not yet available.'
                    )
                    notes += '; TF was not yet available'

                append_result(
                    'tf_delay_check',
                    'FAIL',
                    'startup readiness timeout',
                    notes
                )
                self.shutdown()
                return

        elif self.state == 'recording':
            self.collect_samples()

    def collect_samples(self) -> None:
        self.elapsed_time += self.sample_period
        self.total_sample_cycles += 1

        ros_now = self.now_ros_time_sec()

        for pair in self.selected_pairs:
            parent, child = pair
            tf_msg = self.get_transform(parent, child)

            if tf_msg is None:
                self.get_logger().warn(f'[WARN] Sample lookup failed: {parent} -> {child}')
                continue

            tf_stamp = self.stamp_to_sec(tf_msg.header.stamp.sec, tf_msg.header.stamp.nanosec)
            delay = ros_now - tf_stamp
            self.delay_samples[pair].append(delay)

        current_second = int(self.elapsed_time)
        if current_second > self.last_progress_second:
            self.last_progress_second = current_second

            wall_elapsed = max(
                1e-9,
                time.time() - self.wall_start_time
            ) if self.wall_start_time else 1e-9
            avg_sampler_rate = self.total_sample_cycles / wall_elapsed

            self.get_logger().info(
                f'[INFO] Progress: {min(self.elapsed_time, self.test_duration):.1f}/'
                f'{self.test_duration:.1f} sec | sample cycles: {self.total_sample_cycles} '
                f'| avg sampler rate: {avg_sampler_rate:.2f} Hz'
            )

            for parent, child in self.selected_pairs:
                samples = self.delay_samples[(parent, child)]
                count = len(samples)
                if count > 0:
                    self.get_logger().info(
                        f'[INFO]   {parent} -> {child}: {count} delay samples recorded '
                        f'| latest delay: {samples[-1]:.4f} sec'
                    )

        if self.elapsed_time >= self.test_duration:
            self.analyze_samples()

    def analyze_samples(self) -> None:
        self.get_logger().info('')
        self.get_logger().info('========== ANALYSIS ==========')

        overall_pass = True
        notes_parts = []
        measurement_parts = []

        wall_elapsed = max(
            1e-9,
            time.time() - self.wall_start_time
        ) if self.wall_start_time else 1e-9
        avg_sampler_rate = self.total_sample_cycles / wall_elapsed

        self.get_logger().info(f'[INFO] Total wall time: {wall_elapsed:.3f} sec')
        self.get_logger().info(f'[INFO] Total sample cycles: {self.total_sample_cycles}')
        self.get_logger().info(f'[INFO] Average sampler rate: {avg_sampler_rate:.2f} Hz')

        delay_threshold_sec = 0.10

        for pair in self.selected_pairs:
            parent, child = pair
            delays = self.delay_samples[pair]

            if len(delays) < 2:
                self.get_logger().error(f'[FAIL] Not enough delay samples for {parent} -> {child}')
                overall_pass = False
                notes_parts.append(f'not enough delay samples for {parent}->{child}')
                continue

            avg_delay = sum(delays) / len(delays)
            min_delay = min(delays)
            max_delay = max(delays)

            self.get_logger().info('')
            self.get_logger().info(f'[INFO] Transform checked: {parent} -> {child}')
            self.get_logger().info(f'[INFO] Delay samples: {len(delays)}')
            self.get_logger().info(f'[INFO] Min delay: {min_delay:.4f} sec')
            self.get_logger().info(f'[INFO] Avg delay: {avg_delay:.4f} sec')
            self.get_logger().info(f'[INFO] Max delay: {max_delay:.4f} sec')

            measurement_parts.append(f'{parent}->{child}: {avg_delay:.4f} s avg')

            if abs(avg_delay) > 1000.0:
                self.get_logger().error(
                    '[FAIL] Extremely large TF delay detected. This usually means a clock mismatch.'
                )
                overall_pass = False
                notes_parts.append(f'{parent}->{child} likely clock mismatch')
                continue

            pair_pass = True

            if avg_delay < 0.0:
                self.get_logger().warn(
                    f'[WARN] Average delay is negative for {parent} -> {child}. '
                    'Check clock consistency.'
                )
                notes_parts.append(f'{parent}->{child} negative delay')

            if avg_delay <= delay_threshold_sec:
                self.get_logger().info(
                    f'[PASS] Average TF delay is within threshold ({delay_threshold_sec:.2f} sec)'
                )
            else:
                self.get_logger().error(
                    f'[FAIL] Average TF delay too high: {avg_delay:.4f} sec'
                )
                pair_pass = False
                notes_parts.append(f'{parent}->{child} avg delay high ({avg_delay:.4f} s)')

            if max_delay <= delay_threshold_sec:
                self.get_logger().info(
                    f'[PASS] Max TF delay stayed within threshold ({delay_threshold_sec:.2f} sec)'
                )
            else:
                self.get_logger().error(
                    f'[FAIL] Max TF delay exceeded threshold: {max_delay:.4f} sec'
                )
                pair_pass = False
                notes_parts.append(f'{parent}->{child} max delay high ({max_delay:.4f} s)')

            if not pair_pass:
                overall_pass = False

        self.get_logger().info('')
        self.get_logger().info('========== SUMMARY ==========')

        if overall_pass:
            self.get_logger().info('[PASS] TF delay check passed')
            status = 'PASS'
        else:
            self.get_logger().error('[FAIL] TF delay check failed')
            status = 'FAIL'

        measurement = ' | '.join(measurement_parts) if measurement_parts else 'no delay computed'

        if not notes_parts:
            notes = f'avg sampler rate={avg_sampler_rate:.2f} Hz'
        else:
            notes = '; '.join(notes_parts + [f'avg sampler rate={avg_sampler_rate:.2f} Hz'])

        append_result(
            'tf_delay_check',
            status,
            measurement,
            notes
        )

        self.shutdown()

    def shutdown(self) -> None:
        self.get_logger().info('TF delay check complete. Shutting down.')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TFDelayCheck()

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