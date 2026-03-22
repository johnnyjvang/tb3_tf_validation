#!/usr/bin/env python3

"""
tf_static_check.py

Validate TurtleBot3 static transforms.

Checks:
- base_link -> imu_link
- base_link -> laser OR base_link -> base_scan

For each static transform, sample multiple times and verify:
- translation remains constant
- rotation remains constant

Writes one summary row to the shared TF validation CSV.
"""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from tb3_tf_validation.result_utils import append_result


class TFStaticCheck(Node):
    def __init__(self) -> None:
        super().__init__('tf_static_check')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lidar_candidates = ['laser', 'base_scan']

        self.sample_target = 15
        self.sample_period = 0.2

        self.transform_pairs: List[Tuple[str, str]] = []
        self.samples: Dict[Tuple[str, str], List[TransformStamped]] = {}

        self.started = False
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

    def find_lidar_frame(self) -> Optional[str]:
        for frame in self.lidar_candidates:
            if self.frame_exists(frame):
                return frame
        return None

    def quat_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quat_angle_diff_deg(
        self,
        q1: Tuple[float, float, float, float],
        q2: Tuple[float, float, float, float]
    ) -> float:
        dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]
        dot = max(-1.0, min(1.0, abs(dot)))
        angle_rad = 2.0 * math.acos(dot)
        return math.degrees(angle_rad)

    def translation_diff(
        self,
        a: Tuple[float, float, float],
        b: Tuple[float, float, float]
    ) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def start_check(self) -> None:
        if self.started:
            return
        self.started = True

        self.get_logger().info('========== TF STATIC CHECK ==========')

        self.transform_pairs = []

        if self.frame_exists('imu_link'):
            self.transform_pairs.append(('base_link', 'imu_link'))
            self.get_logger().info('[INFO] Will check static transform: base_link -> imu_link')
        else:
            self.get_logger().warn('[WARN] imu_link frame not found, skipping')

        lidar_frame = self.find_lidar_frame()
        if lidar_frame is not None:
            self.transform_pairs.append(('base_link', lidar_frame))
            self.get_logger().info(
                f'[INFO] Will check static transform: base_link -> {lidar_frame}'
            )
        else:
            self.get_logger().warn('[WARN] No lidar frame found (laser/base_scan), skipping')

        if not self.transform_pairs:
            self.get_logger().error('[FAIL] No static TF pairs available to check.')
            append_result(
                'tf_static_check',
                'FAIL',
                'no static TF pairs checked',
                'imu_link and lidar frame were both unavailable'
            )
            self.shutdown()
            return

        for pair in self.transform_pairs:
            self.samples[pair] = {}

        for pair in self.transform_pairs:
            self.samples[pair] = []

        self.get_logger().info(
            f'[INFO] Collecting {self.sample_target} samples every {self.sample_period:.1f} sec...'
        )

        self.destroy_timer(self.start_timer)
        self.sample_timer = self.create_timer(self.sample_period, self.collect_samples)

    def collect_samples(self) -> None:
        all_done = True

        for parent, child in self.transform_pairs:
            if len(self.samples[(parent, child)]) >= self.sample_target:
                continue

            tf_msg = self.get_transform(parent, child)
            if tf_msg is None:
                self.get_logger().warn(f'[WARN] Sample lookup failed: {parent} -> {child}')
                all_done = False
                continue

            self.samples[(parent, child)].append(tf_msg)

            tx = tf_msg.transform.translation.x
            ty = tf_msg.transform.translation.y
            tz = tf_msg.transform.translation.z
            q = tf_msg.transform.rotation
            yaw_deg = math.degrees(self.quat_to_yaw(q.x, q.y, q.z, q.w))

            self.get_logger().info(
                f'[INFO] {parent} -> {child} sample '
                f'{len(self.samples[(parent, child)]):02d}/{self.sample_target}: '
                f't=({tx:.4f}, {ty:.4f}, {tz:.4f}), yaw={yaw_deg:.2f} deg'
            )

            if len(self.samples[(parent, child)]) < self.sample_target:
                all_done = False

        if all_done:
            self.analyze_samples()

    def analyze_samples(self) -> None:
        if self.sample_timer is not None:
            self.destroy_timer(self.sample_timer)

        self.get_logger().info('')
        self.get_logger().info('========== ANALYSIS ==========')

        overall_pass = True
        notes_parts = []
        stable_pairs = 0

        translation_tol_m = 1e-4
        rotation_tol_deg = 0.05

        for parent, child in self.transform_pairs:
            pair = (parent, child)
            tf_list = self.samples[pair]

            if len(tf_list) < 2:
                self.get_logger().error(f'[FAIL] Not enough samples for {parent} -> {child}')
                overall_pass = False
                notes_parts.append(f'not enough samples for {parent}->{child}')
                continue

            first = tf_list[0]

            first_t = (
                first.transform.translation.x,
                first.transform.translation.y,
                first.transform.translation.z,
            )
            first_q = (
                first.transform.rotation.x,
                first.transform.rotation.y,
                first.transform.rotation.z,
                first.transform.rotation.w,
            )

            max_translation_drift = 0.0
            max_rotation_drift_deg = 0.0

            for tf_msg in tf_list[1:]:
                curr_t = (
                    tf_msg.transform.translation.x,
                    tf_msg.transform.translation.y,
                    tf_msg.transform.translation.z,
                )
                curr_q = (
                    tf_msg.transform.rotation.x,
                    tf_msg.transform.rotation.y,
                    tf_msg.transform.rotation.z,
                    tf_msg.transform.rotation.w,
                )

                t_drift = self.translation_diff(curr_t, first_t)
                q_drift_deg = self.quat_angle_diff_deg(curr_q, first_q)

                max_translation_drift = max(max_translation_drift, t_drift)
                max_rotation_drift_deg = max(max_rotation_drift_deg, q_drift_deg)

            self.get_logger().info(f'[INFO] Transform checked: {parent} -> {child}')
            self.get_logger().info(
                f'[INFO] Max translation drift: {max_translation_drift:.6f} m'
            )
            self.get_logger().info(
                f'[INFO] Max rotation drift: {max_rotation_drift_deg:.6f} deg'
            )

            pair_pass = True

            if max_translation_drift <= translation_tol_m:
                self.get_logger().info(
                    f'[PASS] Translation stable for {parent} -> {child}'
                )
            else:
                self.get_logger().error(
                    f'[FAIL] Translation drift detected for {parent} -> {child}'
                )
                pair_pass = False

            if max_rotation_drift_deg <= rotation_tol_deg:
                self.get_logger().info(
                    f'[PASS] Rotation stable for {parent} -> {child}'
                )
            else:
                self.get_logger().error(
                    f'[FAIL] Rotation drift detected for {parent} -> {child}'
                )
                pair_pass = False

            if pair_pass:
                stable_pairs += 1
            else:
                overall_pass = False
                notes_parts.append(
                    f'{parent}->{child} drift '
                    f'(trans={max_translation_drift:.6f} m, rot={max_rotation_drift_deg:.6f} deg)'
                )

            self.get_logger().info('')

        self.get_logger().info('========== SUMMARY ==========')

        total_pairs = len(self.transform_pairs)
        measurement = f'{stable_pairs}/{total_pairs} static transforms stable'

        if overall_pass:
            self.get_logger().info('[PASS] Static TF transforms are stable')
            status = 'PASS'
        else:
            self.get_logger().error('[FAIL] Static TF validation failed')
            status = 'FAIL'

        if not notes_parts:
            checked_names = [f'{p}->{c}' for p, c in self.transform_pairs]
            notes = 'stable transforms: ' + ', '.join(checked_names)
        else:
            notes = '; '.join(notes_parts)

        append_result(
            'tf_static_check',
            status,
            measurement,
            notes
        )

        self.shutdown()

    def shutdown(self) -> None:
        self.get_logger().info('TF static check complete. Shutting down.')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TFStaticCheck()

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