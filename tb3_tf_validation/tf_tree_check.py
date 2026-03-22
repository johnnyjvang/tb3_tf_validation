#!/usr/bin/env python3

"""
tf_tree_check.py

TurtleBot3-specific TF tree validation.

Checks:
- Required core frames exist
- Expected sensor frames exist
- Accept either 'laser' or 'base_scan' for lidar
- Optional 'map' frame if SLAM/localization is running
- Required transform relationships are resolvable
"""

from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class TFTreeCheck(Node):
    def __init__(self) -> None:
        super().__init__('tf_tree_check')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Core TB3 frames that should almost always exist
        self.required_frames: List[str] = [
            'odom',
            'base_footprint',
            'base_link',
        ]

        # Expected but may vary slightly depending on setup
        self.expected_frames: List[str] = [
            'imu_link',
        ]

        # Optional depending on SLAM/localization
        self.optional_frames: List[str] = [
            'map',
        ]

        # Accept either laser or base_scan for lidar frame
        self.lidar_frame_candidates: List[str] = [
            'laser',
            'base_scan',
        ]

        self.timer = self.create_timer(2.0, self.run_check)
        self.has_run = False

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

    def print_header(self, title: str) -> None:
        self.get_logger().info('')
        self.get_logger().info('=' * 42)
        self.get_logger().info(title)
        self.get_logger().info('=' * 42)

    def run_check(self) -> None:
        if self.has_run:
            return
        self.has_run = True

        self.print_header('TB3 TF TREE CHECK')

        missing_required_frames: List[str] = []
        missing_expected_frames: List[str] = []
        failed_required_links: List[Tuple[str, str]] = []
        warning_links: List[Tuple[str, str]] = []

        # -------- Frame existence --------
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

        # -------- Transform connectivity --------
        self.get_logger().info('')
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

        # IMU link check
        if self.frame_exists('imu_link'):
            if self.transform_exists('base_link', 'imu_link'):
                self.get_logger().info('[PASS] Transform path found: base_link -> imu_link')
            else:
                self.get_logger().warn('[WARN] Transform path missing: base_link -> imu_link')
                warning_links.append(('base_link', 'imu_link'))

        # Lidar link check
        if lidar_frame is not None:
            if self.transform_exists('base_link', lidar_frame):
                self.get_logger().info(f'[PASS] Transform path found: base_link -> {lidar_frame}')
            else:
                self.get_logger().warn(f'[WARN] Transform path missing: base_link -> {lidar_frame}')
                warning_links.append(('base_link', lidar_frame))

        # Optional map link
        if self.frame_exists('map'):
            if self.transform_exists('map', 'odom'):
                self.get_logger().info('[PASS] Transform path found: map -> odom')
            else:
                self.get_logger().warn('[WARN] Transform path missing: map -> odom')
                warning_links.append(('map', 'odom'))
        else:
            self.get_logger().info('[INFO] Skipping map -> odom check because map frame is not present')

        # -------- Summary --------
        self.print_header('SUMMARY')

        if not missing_required_frames and not failed_required_links:
            self.get_logger().info('[PASS] Core TB3 TF tree is valid')
        else:
            self.get_logger().error('[FAIL] Core TB3 TF tree validation failed')

        if missing_required_frames:
            self.get_logger().error('Missing required frames:')
            for frame in missing_required_frames:
                self.get_logger().error(f'  - {frame}')

        if failed_required_links:
            self.get_logger().error('Missing required transform paths:')
            for parent, child in failed_required_links:
                self.get_logger().error(f'  - {parent} -> {child}')

        if missing_expected_frames:
            self.get_logger().warn('Missing expected frames:')
            for frame in missing_expected_frames:
                self.get_logger().warn(f'  - {frame}')

        if warning_links:
            self.get_logger().warn('Non-fatal transform warnings:')
            for parent, child in warning_links:
                self.get_logger().warn(f'  - {parent} -> {child}')

        if lidar_frame is None:
            self.get_logger().warn(
                'No lidar frame detected. Check whether your TB3 is publishing laser/base_scan.'
            )

        self.get_logger().info('=' * 42)
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