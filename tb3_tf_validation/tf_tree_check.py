#!/usr/bin/env python3

"""
tf_tree_check.py

Validate TurtleBot3 TF tree structure.

Checks:
- Required core frames exist
- Expected sensor frames exist
- Accept either 'laser' or 'base_scan' for lidar
- Optional 'map' frame if SLAM/localization is running
- Required transform relationships are resolvable

Writes one summary row to the shared TF validation CSV.
"""

from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from tb3_tf_validation.result_utils import append_result


class TFTreeCheck(Node):
    def __init__(self) -> None:
        super().__init__('tf_tree_check')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.required_frames: List[str] = [
            'odom',
            'base_footprint',
            'base_link',
        ]

        self.expected_frames: List[str] = [
            'imu_link',
        ]

        self.optional_frames: List[str] = [
            'map',
        ]

        self.lidar_frame_candidates: List[str] = [
            'laser',
            'base_scan',
        ]

        self.has_run = False
        self.timer = self.create_timer(2.0, self.run_check)

    def get_all_frames_yaml(self) -> str:
        try:
            return self.tf_buffer.all_frames_as_yaml()
        except Exception:
            return ''

    def frame_exists(self, frame_name: str) -> bool:
        yaml_text = self.get_all_frames_yaml()
        return frame_name in yaml_text

    def find_first_existing_frame(self, frame_names: List[str]) -> Optional[str]:
        for frame in frame_names:
            if self.frame_exists(frame):
                return frame
        return None

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

    def run_check(self) -> None:
        if self.has_run:
            return
        self.has_run = True

        self.get_logger().info('========== TF TREE CHECK ==========')

        missing_required_frames: List[str] = []
        missing_expected_frames: List[str] = []
        failed_required_links: List[Tuple[str, str]] = []
        warning_links: List[Tuple[str, str]] = []

        self.get_logger().info('Checking required frames...')
        for frame in self.required_frames:
            if self.frame_exists(frame):
                self.get_logger().info(f'[PASS] Required frame found: {frame}')
            else:
                self.get_logger().error(f'[FAIL] Missing required frame: {frame}')
                missing_required_frames.append(frame)

        self.get_logger().info('Checking expected frames...')
        for frame in self.expected_frames:
            if self.frame_exists(frame):
                self.get_logger().info(f'[PASS] Expected frame found: {frame}')
            else:
                self.get_logger().warn(f'[WARN] Missing expected frame: {frame}')
                missing_expected_frames.append(frame)

        self.get_logger().info('Checking lidar frame...')
        lidar_frame = self.find_first_existing_frame(self.lidar_frame_candidates)
        if lidar_frame is not None:
            self.get_logger().info(f'[PASS] Lidar frame found: {lidar_frame}')
        else:
            self.get_logger().warn(
                '[WARN] No lidar frame found. Expected one of: '
                + ', '.join(self.lidar_frame_candidates)
            )

        self.get_logger().info('Checking optional frames...')
        for frame in self.optional_frames:
            if self.frame_exists(frame):
                self.get_logger().info(f'[INFO] Optional frame found: {frame}')
            else:
                self.get_logger().info(f'[INFO] Optional frame not present: {frame}')

        self.get_logger().info('Checking required transform paths...')
        required_links: List[Tuple[str, str]] = [
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
        ]

        for parent, child in required_links:
            if self.transform_exists(parent, child):
                self.get_logger().info(f'[PASS] Transform path found: {parent} -> {child}')
            else:
                self.get_logger().error(f'[FAIL] Transform path missing: {parent} -> {child}')
                failed_required_links.append((parent, child))

        if self.frame_exists('imu_link'):
            if self.transform_exists('base_link', 'imu_link'):
                self.get_logger().info('[PASS] Transform path found: base_link -> imu_link')
            else:
                self.get_logger().warn('[WARN] Transform path missing: base_link -> imu_link')
                warning_links.append(('base_link', 'imu_link'))

        if lidar_frame is not None:
            if self.transform_exists('base_link', lidar_frame):
                self.get_logger().info(f'[PASS] Transform path found: base_link -> {lidar_frame}')
            else:
                self.get_logger().warn(f'[WARN] Transform path missing: base_link -> {lidar_frame}')
                warning_links.append(('base_link', lidar_frame))

        if self.frame_exists('map'):
            if self.transform_exists('map', 'odom'):
                self.get_logger().info('[PASS] Transform path found: map -> odom')
            else:
                self.get_logger().warn('[WARN] Transform path missing: map -> odom')
                warning_links.append(('map', 'odom'))
        else:
            self.get_logger().info('[INFO] Skipping map -> odom check because map frame is not present')

        self.get_logger().info('========== SUMMARY ==========')

        overall_pass = not missing_required_frames and not failed_required_links

        if overall_pass:
            self.get_logger().info('[PASS] Core TB3 TF tree is valid')
            status = 'PASS'
            measurement = 'core TF tree valid'
        else:
            self.get_logger().error('[FAIL] Core TB3 TF tree validation failed')
            status = 'FAIL'
            measurement = 'core TF tree invalid'

        notes_parts = []

        if missing_required_frames:
            notes_parts.append(
                'missing required frames: ' + ', '.join(missing_required_frames)
            )

        if failed_required_links:
            notes_parts.append(
                'missing required links: ' + ', '.join(
                    [f'{p}->{c}' for p, c in failed_required_links]
                )
            )

        if missing_expected_frames:
            notes_parts.append(
                'missing expected frames: ' + ', '.join(missing_expected_frames)
            )

        if warning_links:
            notes_parts.append(
                'warning links: ' + ', '.join(
                    [f'{p}->{c}' for p, c in warning_links]
                )
            )

        if lidar_frame is None:
            notes_parts.append('no lidar frame found')

        if not notes_parts:
            notes_parts.append('all core TF checks passed')

        notes = '; '.join(notes_parts)

        append_result(
            'tf_tree_check',
            status,
            measurement,
            notes
        )

        self.get_logger().info('TF tree check complete. Shutting down.')
        self.destroy_timer(self.timer)
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TFTreeCheck()

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