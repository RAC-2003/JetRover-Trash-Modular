"""Odometry position tracking + closed-loop go-to-pose navigation.

This is the ONE implementation of what was navigate_to_odom_position() in the
old project - independently copy-pasted across trash_sorter.py, V5.py and
home_to_bin.py with silent drift between copies. Everything that needs
closed-loop navigation (the pipeline, scripts/calibrate_home_bin.py, test
nodes) calls this same class.

Heading source: wheel odometry yaw is unreliable during in-place spins on
this base (wheel slip), so headings during a maneuver are tracked via the
IMU's integrated yaw delta, anchored to the wheel-odom yaw at maneuver start.
"""

import math
import time

from nav_msgs.msg import Odometry

from trash_modular.utils.transforms import normalize_deg, quaternion_to_yaw_deg

ROTATE_IN_PLACE_THRESHOLD_DEG = 45.0
IMU_RESYNC_INTERVAL_S = 5.0


class PositionTracker:
    def __init__(self, node, config, logger=None):
        self.node = node
        self.logger = logger or node.get_logger()

        nav_cfg = config.get('navigation', {})
        topic = config.get('robot', {}).get('odom_topic', '/odom')
        self.pos_tolerance_m = float(nav_cfg.get('position_tolerance_m', 0.05))
        self.yaw_tolerance_deg = float(nav_cfg.get('yaw_tolerance_deg', 3.0))
        self.linear_speed = float(nav_cfg.get('linear_speed', 0.1))
        self.angular_speed = float(nav_cfg.get('angular_speed', 0.3))
        self.timeout_s = float(nav_cfg.get('timeout_s', 60.0))

        self._x = None
        self._y = None
        self._yaw_deg = 0.0
        node.create_subscription(Odometry, topic, self._callback, 10)
        self.logger.info(f'PositionTracker ready - subscribed to {topic}')

    def _callback(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._yaw_deg = quaternion_to_yaw_deg(msg.pose.pose.orientation)

    def is_ready(self):
        return self._x is not None

    def current_pose(self):
        """Returns (x, y, yaw_deg) or None if no odom received yet."""
        if not self.is_ready():
            return None
        return self._x, self._y, self._yaw_deg

    def go_to_pose(self, base, imu, target_x, target_y, target_yaw_deg, running_flag=None):
        """Closed-loop drive+steer to (target_x, target_y, target_yaw_deg) in the
        odom frame. Returns True if position (and final heading) was reached."""
        if not self.is_ready():
            self.logger.error('PositionTracker.go_to_pose: no odom yet - cannot navigate')
            return False

        use_imu = imu.is_ready()
        if not use_imu:
            self.logger.warn('PositionTracker.go_to_pose: IMU not ready - falling back to wheel-odom heading')

        start_wheel_yaw = self._yaw_deg
        start_imu_yaw = imu.raw_integrated_yaw_deg()
        last_resync = time.time()

        deadline = time.time() + self.timeout_s
        last_log = 0.0
        rotate_burst_start = time.time()
        stall_check_time = time.time()
        stall_check_yaw = None
        reached_position = False

        while time.time() < deadline:
            if running_flag is not None and not running_flag():
                break

            cur_x, cur_y = self._x, self._y
            cur_yaw = self._corrected_yaw(imu, use_imu, start_wheel_yaw, start_imu_yaw)

            dx, dy = target_x - cur_x, target_y - cur_y
            dist = math.hypot(dx, dy)
            if dist < self.pos_tolerance_m:
                base.stop()
                self.logger.info(f'PositionTracker: position reached ({cur_x:.3f},{cur_y:.3f})')
                reached_position = True
                break

            heading_error = normalize_deg(math.degrees(math.atan2(dy, dx)) - cur_yaw)

            if time.time() - last_log > 1.0:
                self.logger.info(
                    f'PositionTracker: dist={dist:.3f}m heading_err={heading_error:.1f}deg '
                    f'pos=({cur_x:.3f},{cur_y:.3f}) yaw={cur_yaw:.1f}deg'
                )
                last_log = time.time()

            if abs(heading_error) > ROTATE_IN_PLACE_THRESHOLD_DEG:
                stall_check_yaw, stall_check_time = self._handle_stall_and_burst(
                    base, cur_yaw, stall_check_yaw, stall_check_time, rotate_burst_start
                )
                if time.time() - rotate_burst_start > 3.0:
                    base.stop()
                    time.sleep(0.3)
                    rotate_burst_start = time.time()
                base.move(angular=self.angular_speed if heading_error > 0 else -self.angular_speed)
            else:
                stall_check_yaw = None
                rotate_burst_start = time.time()
                if use_imu and time.time() - last_resync > IMU_RESYNC_INTERVAL_S:
                    start_wheel_yaw = self._yaw_deg
                    start_imu_yaw = imu.raw_integrated_yaw_deg()
                    last_resync = time.time()
                angular = max(-self.angular_speed, min(self.angular_speed, math.radians(heading_error) * 1.5))
                speed_scale = max(0.4, 1.0 - abs(heading_error) / ROTATE_IN_PLACE_THRESHOLD_DEG)
                base.move(linear=self.linear_speed * speed_scale, angular=angular)

            time.sleep(0.05)
        else:
            self.logger.warn('PositionTracker.go_to_pose: timed out before reaching position')

        base.stop()
        time.sleep(0.3)

        if not reached_position:
            self.logger.warn('PositionTracker.go_to_pose: navigation failed - skipping final heading correction')
            return False

        self._final_heading_correction(base, imu, target_yaw_deg)
        self.logger.info('PositionTracker.go_to_pose: navigation complete')
        return True

    def _corrected_yaw(self, imu, use_imu, start_wheel_yaw, start_imu_yaw):
        if not use_imu:
            return self._yaw_deg
        return normalize_deg(start_wheel_yaw + (imu.raw_integrated_yaw_deg() - start_imu_yaw))

    def _handle_stall_and_burst(self, base, cur_yaw, stall_check_yaw, stall_check_time, rotate_burst_start):
        if stall_check_yaw is None:
            return cur_yaw, time.time()
        if time.time() - stall_check_time > 2.0:
            yaw_progress = abs(cur_yaw - stall_check_yaw)
            yaw_progress = 360 - yaw_progress if yaw_progress > 180 else yaw_progress
            if yaw_progress < 3.0:
                self.logger.warn(f'PositionTracker: rotation stalled ({yaw_progress:.1f}deg/2s) - pausing to release')
                base.stop()
                time.sleep(0.6)
            return cur_yaw, time.time()
        return stall_check_yaw, stall_check_time

    def _final_heading_correction(self, base, imu, target_yaw_deg):
        use_imu = imu.is_ready()
        start_wheel_yaw = self._yaw_deg
        start_imu_yaw = imu.raw_integrated_yaw_deg()

        def corrected_yaw():
            return self._corrected_yaw(imu, use_imu, start_wheel_yaw, start_imu_yaw)

        yaw_error = normalize_deg(target_yaw_deg - corrected_yaw())
        if abs(yaw_error) <= self.yaw_tolerance_deg:
            return

        self.logger.info(f'PositionTracker: final heading correction {yaw_error:.1f}deg')
        start_yaw = corrected_yaw()
        angular = self.angular_speed if yaw_error > 0 else -self.angular_speed
        correction_deadline = time.time() + abs(yaw_error) / (self.angular_speed * 57.3) * 3.0
        while time.time() < correction_deadline:
            if abs(corrected_yaw() - start_yaw) >= abs(yaw_error) - self.yaw_tolerance_deg:
                break
            # Re-sent every iteration - a single move() before this loop gets
            # zeroed by RobotBase's watchdog partway through the correction.
            base.move(angular=angular)
            time.sleep(0.02)
        base.stop()


def resolve_offset_target(home_x, home_y, home_yaw_deg, offset):
    """offset: {'dx':..,'dy':..,'dyaw_deg':..} relative to home, as stored in
    config bins.targets / calibrate_home_bin.py's calibration file. This is
    the fix from the old home_to_bin.py that never made it into trash_sorter.py:
    /odom re-zeros every session, so bin positions must be stored relative to
    home, not as stale absolute odom coordinates."""
    target_x = home_x + offset['dx']
    target_y = home_y + offset['dy']
    target_yaw = normalize_deg(home_yaw_deg + offset['dyaw_deg'])
    return target_x, target_y, target_yaw
