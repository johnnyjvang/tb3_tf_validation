#!/usr/bin/env python3

"""
tf_motion_consistency.py

Validate that TurtleBot3 TF motion is consistent with commanded motion.

Sequence:
1. Wait for odom and TF readiness
2. Record initial TF pose
3. Command forward motion
4. Stop and measure TF translation change
5. Command in-place rotation
6. Stop and measure TF yaw change

Checks:
- Forward command should produce noticeable TF translation
- Positive forward command should produce positive x motion
- Positive angular command should produce positive yaw change

Uses TwistStamped on /cmd_vel to match the TurtleBot3 setup.
Writes one summary row to the shared TF validation CSV.
"""

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from tb3_tf_validation.result_utils import append_result


# ===== Topics =====
ODOM_TOPIC = '/odom'
CMD_VEL_TOPIC = '/cmd_vel'

# ===== Timing =====
CONTROL_PERIOD = 0.05
PROGRESS_PERIOD = 1.0
MAX_TEST_TIME = 30.0
STARTUP_WAIT_TIMEOUT = 10.0
SETTLE_TIME = 1.0

# ===== Motion Settings =====
FORWARD_SPEED = 0.08       # m/s
FORWARD_DURATION = 2.0     # sec

ROTATE_SPEED = 0.40        # rad/s
ROTATE_DURATION = 2.0      # sec

# ===== Pass Thresholds =====
MIN_FORWARD_DISTANCE = 0.05       # m
MIN_ROTATION_DEG = 10.0           # deg


class TFMotionConsistency(Node):
    def __init__(self) -> None:
        super().__init__('tf_motion_consistency')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)

        self.target_pairs = [
            ('odom', 'base_footprint'),
            ('odom', 'base_link'),
        ]
        self.selected_pair: Optional[Tuple[str, str]] = None

        # Timing / state
        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        self.phase = 'wait_for_ready'
        self.phase_start_time = time.time()

        # Odom readiness tracking
        self.msg_count = 0
        self.have_odom = False
        self.current_odom_pose = None

        # TF pose snapshots
        self.start_pose = None
        self.after_forward_pose = None
        self.after_rotate_pose = None

        # Timers
        self.timer = self.create_timer(CONTROL_PERIOD, self.loop)
        self.progress_timer = self.create_timer(PROGRESS_PERIOD, self.progress_update)

        self.get_logger().info('========== TF MOTION CONSISTENCY CHECK ==========')
        self.get_logger().info(f'Odometry topic: {ODOM_TOPIC}')
        self.get_logger().info(f'Command topic: {CMD_VEL_TOPIC}')
        self.get_logger().info(f'Forward speed: {FORWARD_SPEED:.2f} m/s')
        self.get_logger().info(f'Forward duration: {FORWARD_DURATION:.2f} sec')
        self.get_logger().info(f'Rotate speed: {ROTATE_SPEED:.2f} rad/s')
        self.get_logger().info(f'Rotate duration: {ROTATE_DURATION:.2f} sec')

    def odom_cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.current_odom_pose = {'x': x, 'y': y}
        self.have_odom = True
        self.msg_count += 1

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        cmd = TwistStamped()
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def stop_robot(self) -> None:
        self.publish_cmd(0.0, 0.0)

    def progress_update(self) -> None:
        if self.done:
            return

        elapsed = time.time() - self.start_time
        phase_elapsed = time.time() - self.phase_start_time

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {MAX_TEST_TIME:.1f}s | '
            f'phase: {self.phase} | '
            f'phase_elapsed: {phase_elapsed:.1f}s | '
            f'odom_msgs: {self.msg_count}'
        )

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

    def choose_tf_pair(self) -> bool:
        for parent, child in self.target_pairs:
            if self.get_transform(parent, child) is not None:
                self.selected_pair = (parent, child)
                self.get_logger().info(f'[PASS] Using TF pair: {parent} -> {child}')
                return True
        return False

    def quat_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        if self.selected_pair is None:
            return None

        tf_msg = self.get_transform(self.selected_pair[0], self.selected_pair[1])
        if tf_msg is None:
            return None

        x = tf_msg.transform.translation.x
        y = tf_msg.transform.translation.y
        q = tf_msg.transform.rotation
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        return (x, y, yaw)

    def transition(self, new_phase: str) -> None:
        self.phase = new_phase
        self.phase_start_time = time.time()
        self.get_logger().info(f'[INFO] Transition -> {new_phase}')

    def finish_and_exit(self, status: str, measurement: str, notes: str) -> None:
        self.stop_robot()

        if status == 'PASS':
            self.get_logger().info(f'Result: {status}')
        else:
            self.get_logger().error(f'Result: {status}')

        append_result(
            test_name='tf_motion_consistency',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def analyze_and_finish(self) -> None:
        self.get_logger().info('')
        self.get_logger().info('========== ANALYSIS ==========')

        if self.start_pose is None or self.after_forward_pose is None or self.after_rotate_pose is None:
            self.finish_and_exit(
                'FAIL',
                'missing pose snapshots',
                'missing TF pose snapshots for analysis'
            )
            return

        x0, y0, yaw0 = self.start_pose
        x1, y1, yaw1 = self.after_forward_pose
        x2, y2, yaw2 = self.after_rotate_pose

        dx_forward = x1 - x0
        dy_forward = y1 - y0
        dist_forward = math.sqrt(dx_forward ** 2 + dy_forward ** 2)

        dyaw_forward = self.normalize_angle(yaw1 - yaw0)
        dyaw_rotate = self.normalize_angle(yaw2 - yaw1)
        dyaw_rotate_deg = math.degrees(dyaw_rotate)

        self.get_logger().info(
            f'[INFO] Forward TF delta: dx={dx_forward:.3f} m, '
            f'dy={dy_forward:.3f} m, dist={dist_forward:.3f} m'
        )
        self.get_logger().info(
            f'[INFO] Yaw change during forward: {math.degrees(dyaw_forward):.2f} deg'
        )
        self.get_logger().info(
            f'[INFO] Yaw change during rotate: {dyaw_rotate_deg:.2f} deg'
        )

        overall_pass = True
        notes_parts = []

        if dist_forward >= MIN_FORWARD_DISTANCE:
            self.get_logger().info('[PASS] Forward command produced noticeable TF translation')
        else:
            self.get_logger().error('[FAIL] Forward command did not produce enough TF translation')
            overall_pass = False
            notes_parts.append(f'forward distance too small ({dist_forward:.3f} m)')

        if dx_forward > 0.0:
            self.get_logger().info('[PASS] Forward command produced positive x motion in TF')
        else:
            self.get_logger().warn('[WARN] Forward command did not produce positive x motion')
            overall_pass = False
            notes_parts.append(f'forward dx not positive ({dx_forward:.3f} m)')

        if abs(dyaw_rotate_deg) >= MIN_ROTATION_DEG:
            self.get_logger().info('[PASS] Rotate command produced noticeable yaw change in TF')
        else:
            self.get_logger().error('[FAIL] Rotate command did not produce enough yaw change')
            overall_pass = False
            notes_parts.append(f'rotation too small ({dyaw_rotate_deg:.2f} deg)')

        if dyaw_rotate > 0.0:
            self.get_logger().info('[PASS] Positive angular command produced positive yaw change')
        else:
            self.get_logger().warn('[WARN] Positive angular command did not produce positive yaw change')
            overall_pass = False
            notes_parts.append(f'rotation sign incorrect ({dyaw_rotate_deg:.2f} deg)')

        self.get_logger().info('')
        self.get_logger().info('========== SUMMARY ==========')

        measurement = (
            f'forward={dist_forward:.3f} m | rotate={dyaw_rotate_deg:.2f} deg'
        )

        if not notes_parts:
            notes = (
                f'dx={dx_forward:.3f} m, dy={dy_forward:.3f} m, '
                f'forward_yaw={math.degrees(dyaw_forward):.2f} deg'
            )
        else:
            notes = '; '.join(notes_parts)

        if overall_pass:
            self.get_logger().info('[PASS] TF motion consistency check passed')
            self.finish_and_exit('PASS', measurement, notes)
        else:
            self.get_logger().error('[FAIL] TF motion consistency check failed')
            self.finish_and_exit('FAIL', measurement, notes)

    def loop(self) -> None:
        now = time.time()
        elapsed = now - self.start_time
        phase_elapsed = now - self.phase_start_time

        if self.done:
            if now - self.finish_time > 0.5:
                self.get_logger().info('Exiting tf_motion_consistency')
                rclpy.shutdown()
            return

        if elapsed > MAX_TEST_TIME:
            self.finish_and_exit(
                'FAIL',
                'timed out',
                'motion consistency test timed out'
            )
            return

        if self.phase == 'wait_for_ready':
            self.stop_robot()

            tf_ready = self.choose_tf_pair() if self.selected_pair is None else True

            if self.have_odom and tf_ready:
                self.start_pose = self.get_pose()
                if self.start_pose is None:
                    return

                x, y, yaw = self.start_pose
                self.get_logger().info(
                    f'[INFO] Start pose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.2f} deg'
                )
                self.transition('moving_forward')
                return

            if phase_elapsed >= STARTUP_WAIT_TIMEOUT:
                self.finish_and_exit(
                    'FAIL',
                    'startup readiness timeout',
                    'timed out waiting for odom and TF readiness'
                )
                return
            return

        if self.phase == 'moving_forward':
            if phase_elapsed < FORWARD_DURATION:
                self.publish_cmd(FORWARD_SPEED, 0.0)
            else:
                self.stop_robot()
                self.transition('settling_after_forward')
            return

        if self.phase == 'settling_after_forward':
            self.stop_robot()
            if phase_elapsed >= SETTLE_TIME:
                self.after_forward_pose = self.get_pose()
                if self.after_forward_pose is None:
                    self.finish_and_exit(
                        'FAIL',
                        'missing post-forward TF pose',
                        'could not record post-forward TF pose'
                    )
                    return

                x, y, yaw = self.after_forward_pose
                self.get_logger().info(
                    f'[INFO] Post-forward pose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.2f} deg'
                )
                self.transition('rotating')
            return

        if self.phase == 'rotating':
            if phase_elapsed < ROTATE_DURATION:
                self.publish_cmd(0.0, ROTATE_SPEED)
            else:
                self.stop_robot()
                self.transition('settling_after_rotate')
            return

        if self.phase == 'settling_after_rotate':
            self.stop_robot()
            if phase_elapsed >= SETTLE_TIME:
                self.after_rotate_pose = self.get_pose()
                if self.after_rotate_pose is None:
                    self.finish_and_exit(
                        'FAIL',
                        'missing post-rotate TF pose',
                        'could not record post-rotate TF pose'
                    )
                    return

                x, y, yaw = self.after_rotate_pose
                self.get_logger().info(
                    f'[INFO] Post-rotate pose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.2f} deg'
                )
                self.analyze_and_finish()
            return


def main(args=None):
    rclpy.init(args=args)
    node = TFMotionConsistency()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()