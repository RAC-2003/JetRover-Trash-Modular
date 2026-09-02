"""Movement primitives: forward/backward, rotate-by-angle, drive-until-lidar,
stop. Built on hardware.base.RobotBase (for the actual velocity command) plus
perception.imu / perception.lidar (for closed-loop feedback). No pixel/VLM
logic lives here - see navigation/alignment.py for that.
"""

import time


class Movement:
    def __init__(self, base, imu, lidar, config, logger=None):
        self.base = base
        self.imu = imu
        self.lidar = lidar
        self.logger = logger

        nav_cfg = config.get('navigation', {})
        self.default_angular_speed = float(nav_cfg.get('angular_speed', 0.3))
        self.default_linear_speed = float(nav_cfg.get('linear_speed', 0.1))
        self.lost_target_margin_m = float(nav_cfg.get('lost_target_margin_m', 0.04))

    def stop(self):
        self.base.stop()

    def forward(self, speed=None, duration_s=1.0, running_flag=None):
        self._hold(linear=(speed if speed is not None else self.default_linear_speed), duration_s=duration_s, running_flag=running_flag)

    def backward(self, speed=None, duration_s=1.0, running_flag=None):
        self._hold(linear=-(speed if speed is not None else self.default_linear_speed), duration_s=duration_s, running_flag=running_flag)

    def _hold(self, linear=0.0, angular=0.0, duration_s=1.0, running_flag=None):
        """Re-sends the command every tick for duration_s - a single move()
        call would get zeroed by RobotBase's watchdog after watchdog_timeout_s
        (0.5s default) for any duration longer than that."""
        deadline = time.time() + duration_s
        while time.time() < deadline:
            if running_flag is not None and not running_flag():
                break
            self.base.move(linear=linear, angular=angular)
            time.sleep(0.05)
        self.base.stop()

    def rotate_by_angle(self, angle_deg, angular_speed=None, running_flag=None):
        """Rotate in place by angle_deg (signed) using IMU-integrated heading.
        running_flag: optional callable, checked each iteration; return False to abort early."""
        if abs(angle_deg) < 1.0:
            return True
        if not self.imu.is_ready():
            if self.logger:
                self.logger.error('Movement.rotate_by_angle: IMU not ready - refusing to rotate blind')
            return False

        angular_speed = angular_speed if angular_speed is not None else self.default_angular_speed
        import math
        target_rad = abs(math.radians(angle_deg))
        direction = 1.0 if angle_deg > 0 else -1.0
        start_yaw = self.imu.raw_integrated_yaw_deg()

        timeout = time.time() + target_rad / angular_speed * 3.0
        reached = False
        while True:
            if running_flag is not None and not running_flag():
                break
            rotated = abs(math.radians(self.imu.raw_integrated_yaw_deg() - start_yaw))
            if rotated >= target_rad:
                reached = True
                break
            if time.time() > timeout:
                if self.logger:
                    self.logger.warn(f'Movement.rotate_by_angle: timeout, rotated={math.degrees(rotated):.1f}deg')
                break
            # Re-sent every iteration (not just once before the loop) so
            # hardware.base.RobotBase's command watchdog doesn't zero the
            # velocity mid-turn - it was killing motion after watchdog_timeout_s
            # (0.5s) while this loop kept polling IMU thinking it was still moving.
            self.base.move(angular=direction * angular_speed)
            time.sleep(0.02)
        self.base.stop()
        time.sleep(0.2)
        return reached

    def drive_until_lidar(self, target_distance_m, speed=0.1, hard_timeout_s=30.0, running_flag=None):
        """Drive forward closed-loop until the front LiDAR reads target_distance_m.

        Small objects (an apple, tissue paper) can drop out of a 2D LiDAR's
        scan plane once the robot is close enough - the beam then reports
        whatever is behind the object (a wall, etc), which reads as a LARGER
        distance even though the robot is still driving straight at the
        object. Without a guard, drive_until_lidar would trust that larger
        reading and keep driving, ramming through where the object actually
        is. So: track the closest distance actually seen, and if a later
        reading jumps back up past that (by more than lost_target_margin_m),
        treat the object as lost-at-close-range and stop rather than
        continuing to close a gap that isn't real.
        """
        if not self.lidar.is_ready():
            if self.logger:
                self.logger.error('Movement.drive_until_lidar: LiDAR not ready - refusing to drive blind')
            return False

        deadline = time.time() + hard_timeout_s
        reached = False
        min_distance_seen = None
        while time.time() < deadline:
            if running_flag is not None and not running_flag():
                break
            d = self.lidar.front_distance()
            if d is not None:
                if self.logger:
                    self.logger.info(f'Movement.drive_until_lidar: {d:.3f}m target={target_distance_m:.3f}m')
                if d <= target_distance_m:
                    reached = True
                    break
                if min_distance_seen is not None and d > min_distance_seen + self.lost_target_margin_m:
                    if self.logger:
                        self.logger.warn(
                            f'Movement.drive_until_lidar: reading jumped to {d:.3f}m after '
                            f'getting as close as {min_distance_seen:.3f}m - object likely lost '
                            'at close range, stopping here rather than driving further'
                        )
                    reached = True
                    break
                if min_distance_seen is None or d < min_distance_seen:
                    min_distance_seen = d
                if d < 0.25 and speed > 0.02:
                    speed = 0.02
            # Re-sent every iteration - see the same note in rotate_by_angle
            # above about RobotBase's watchdog otherwise zeroing this mid-drive.
            self.base.move(linear=speed)
            time.sleep(0.05)
        self.base.stop()
        if not reached and self.logger:
            self.logger.warn('Movement.drive_until_lidar: did not reach target distance before timeout')
        return reached
