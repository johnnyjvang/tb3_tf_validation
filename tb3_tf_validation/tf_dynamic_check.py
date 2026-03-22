#!/usr/bin/env python3

"""
tf_dynamic_check.py

Validate that the TurtleBot3 dynamic transform is updating over time.

Primary check:
- odom -> base_footprint

Fallback:
- odom -> base_link

Checks:
- Transform lookup success
- Timestamp freshness
- Whether TF updates over repeated samples
- Whether the transform appears frozen

Writes one summary row to the shared TF validation CSV.
"""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from tb3_tf_validation.result_utils import append_result


class TFDynamicCheck(Node):
    def __init__(self) -> None:
        super().__init__('tf_dynamic_check')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pairs: List[Tuple[str, str]] = [
            ('odom', 'base_footprint'),
            ('odom', 'base_link'),
        ]

        self.selected_pair: Optional[Tuple[str, str]] = None
        self.samples: List[TransformStamped] = []

        self.sample_count_target = 20
        self.sample_period = 0.2

        self.started = False
        self.start_timer = self.create_timer(2.0, self.start_check)
        self.sample_timer = None

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

    def quat_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def pose_summary(self, tf_msg: TransformStamped) -> Tuple[float, float, float]:
        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        q = tf_msg.transform.rotation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        return tx, ty, yaw

    def stamp_to_sec(self, tf_msg: TransformStamped) -> float:
        return float(tf_msg.header.stamp.sec) + float(tf_msg.header.stamp.nanosec) * 1e-9

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def start_check(self) -> None:
        if self.started:
            return
        self.started = True

        self.get_logger().info('========== TF DYNAMIC CHECK ==========')
        self.get_logger().info('Searching for dynamic TF pair...')

        for parent, child in self.target_pairs:
            if self.transform_exists(parent, child):
                self.selected_pair = (parent, child)
                self.get_logger().info(f'[PASS] Using transform pair: {parent} -> {child}')
                break

        if self.selected_pair is None:
            self.get_logger().error(
                '[FAIL] Could not resolve odom -> base_footprint or odom -> base_link'
            )
            append_result(
                'tf_dynamic_check',
                'FAIL',
                'no dynamic TF pair resolved',
                'could not resolve odom->base_footprint or odom->base_link'
            )
            self.shutdown()
            return

        self.get_logger().info(
            f'[INFO] Collecting {self.sample_count_target} samples every {self.sample_period:.1f} sec...'
        )

        self.destroy_timer(self.start_timer)
        self.sample_timer = self.create_timer(self.sample_period, self.collect_sample)

    def collect_sample(self) -> None:
        if self.selected_pair is None:
            self.get_logger().error('[FAIL] No transform pair selected.')
            append_result(
                'tf_dynamic_check',
                'FAIL',
                'no transform pair selected',
                'internal script error before sampling'
            )
            self.shutdown()
            return

        parent, child = self.selected_pair
        tf_msg = self.get_transform(parent, child)

        if tf_msg is None:
            self.get_logger().warn(f'[WARN] Sample lookup failed: {parent} -> {child}')
            return

        self.samples.append(tf_msg)

        tx, ty, yaw = self.pose_summary(tf_msg)
        stamp_sec = self.stamp_to_sec(tf_msg)

        self.get_logger().info(
            f'[INFO] Sample {len(self.samples):02d}/{self.sample_count_target}: '
            f't=({tx:.3f}, {ty:.3f}), yaw={math.degrees(yaw):.2f} deg, stamp={stamp_sec:.3f}'
        )

        if len(self.samples) >= self.sample_count_target:
            self.analyze_samples()

    def analyze_samples(self) -> None:
        if self.sample_timer is not None:
            self.destroy_timer(self.sample_timer)

        self.get_logger().info('')
        self.get_logger().info('========== ANALYSIS ==========')

        if len(self.samples) < 2:
            self.get_logger().error('[FAIL] Not enough valid samples collected.')
            append_result(
                'tf_dynamic_check',
                'FAIL',
                'not enough samples',
                f'only collected {len(self.samples)} valid samples'
            )
            self.shutdown()
            return

        stamps = [self.stamp_to_sec(s) for s in self.samples]
        poses = [self.pose_summary(s) for s in self.samples]

        first_x, first_y, first_yaw = poses[0]
        last_x, last_y, last_yaw = poses[-1]

        dt_stamp = stamps[-1] - stamps[0]
        dx = last_x - first_x
        dy = last_y - first_y
        dist = math.sqrt(dx * dx + dy * dy)
        dyaw = self.normalize_angle(last_yaw - first_yaw)

        unique_stamps = len(set(round(s, 6) for s in stamps))
        repeated_stamps = len(stamps) - unique_stamps

        self.get_logger().info(
            f'[INFO] Transform pair tested: {self.selected_pair[0]} -> {self.selected_pair[1]}'
        )
        self.get_logger().info(f'[INFO] Header stamp span: {dt_stamp:.3f} sec')
        self.get_logger().info(f'[INFO] Position change: dx={dx:.3f} m, dy={dy:.3f} m, dist={dist:.3f} m')
        self.get_logger().info(f'[INFO] Yaw change: {math.degrees(dyaw):.2f} deg')
        self.get_logger().info(f'[INFO] Repeated timestamp count: {repeated_stamps}')

        pass_dynamic = True
        notes_parts = []

        if dt_stamp > 0.0:
            self.get_logger().info('[PASS] TF timestamps are updating over time')
        else:
            self.get_logger().error('[FAIL] TF timestamps are not updating')
            pass_dynamic = False
            notes_parts.append('timestamps did not advance')

        if unique_stamps > 1:
            self.get_logger().info('[PASS] TF does not appear frozen by timestamp')
        else:
            self.get_logger().error('[FAIL] TF appears frozen (all timestamps identical)')
            pass_dynamic = False
            notes_parts.append('all timestamps identical')

        pose_note = (
            f'dist={dist:.3f} m, yaw_change={math.degrees(dyaw):.2f} deg'
        )

        if dist > 0.01 or abs(math.degrees(dyaw)) > 1.0:
            self.get_logger().info('[PASS] Pose changed during sampling (robot likely moved)')
        else:
            self.get_logger().info(
                '[INFO] Pose change was very small. This is acceptable if the robot was stationary.'
            )
            notes_parts.append('pose change was small; robot may have been stationary')

        self.get_logger().info('')
        self.get_logger().info('========== SUMMARY ==========')

        measurement = f'{self.selected_pair[0]}->{self.selected_pair[1]} updating'

        if pass_dynamic:
            self.get_logger().info('[PASS] Dynamic TF is updating correctly')
            status = 'PASS'
        else:
            self.get_logger().error('[FAIL] Dynamic TF check failed')
            status = 'FAIL'

        if not notes_parts:
            notes = pose_note
        else:
            notes = '; '.join(notes_parts + [pose_note])

        append_result(
            'tf_dynamic_check',
            status,
            measurement,
            notes
        )

        self.shutdown()

    def shutdown(self) -> None:
        self.get_logger().info('TF dynamic check complete. Shutting down.')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TFDynamicCheck()

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