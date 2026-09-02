"""Bin delivery: navigate to the material's bin then drop the held object.

Two ways to find the bin, selected by config bins.location_mode:
  static  - drive to a home-relative odom offset (not a stale absolute odom
            coordinate - see navigation.position.resolve_offset_target).
  detect  - rotate in place searching for the bin visually (same VLM client
            as trash detection, perception/bin_detector.py), then approach
            with the LiDAR. Mirrors how the trash object itself is found.
"""

import math
import time

from trash_modular.navigation.position import resolve_offset_target
from trash_modular.utils.transforms import pixel_offset_to_angle_deg


class PlaceExecutor:
    def __init__(self, arm, gripper, movement, position, config, logger=None, camera=None, bin_detector=None):
        self.arm = arm
        self.gripper = gripper
        self.movement = movement
        self.position = position
        self.logger = logger
        self.camera = camera
        self.bin_detector = bin_detector

        nav_cfg = config.get('navigation', {})
        self.drop_lidar_distance_m = float(nav_cfg.get('drop_lidar_distance_m', 0.150))
        # Guard for the optional post-odom LiDAR nudge: once the robot is within
        # this distance of the configured bin coordinate, stop - never let a
        # LiDAR approach carry it past the manually-set bin position.
        self.drop_overrun_margin_m = float(nav_cfg.get('drop_overrun_margin_m', 0.10))

        bins_cfg = config.get('bins', {})
        self.location_mode = bins_cfg.get('location_mode', 'static')
        self.targets = {t['material']: t['offset'] for t in bins_cfg.get('targets', [])}

        search_cfg = bins_cfg.get('search', {})
        self.search_step_deg = float(search_cfg.get('step_deg', 30))
        self.search_pause_s = float(search_cfg.get('pause_s', 1.5))
        self.search_max_steps = int(search_cfg.get('max_steps', 12))
        self.search_pixel_tolerance = float(search_cfg.get('pixel_tolerance', 40))
        self.search_align_max_steps = int(search_cfg.get('align_max_steps', 6))
        self.search_outlier_px = float(search_cfg.get('outlier_px', 150))

        cam_cfg = config.get('camera', {})
        self.cam_center_x = float(cam_cfg.get('rgb_center_x', 324.88))
        self.cam_focal_x = float(cam_cfg.get('rgb_focal_x', 357.69))

        # Same turret gain the object's fine_align uses for its joint1
        # correction - reused here for the post-approach arm recentre.
        align_cfg = config.get('alignment', {})
        self.pixels_per_pulse = float(align_cfg.get('pixels_per_pulse', 2.2))
        self.recenter_max_iterations = int(search_cfg.get('recenter_max_iterations', 3))

        # Set by _navigate_to_bin_by_detection's arm recentre, consumed once
        # by drop() so the recentred turret angle survives into the drop
        # pose instead of snapping back to the drop pose's fixed joint1.
        self._recentered_joint1 = None

        if self.location_mode == 'detect' and (camera is None or bin_detector is None):
            if self.logger:
                self.logger.error(
                    'PlaceExecutor: bins.location_mode is "detect" but no camera/bin_detector was '
                    'given - falling back to static offset navigation'
                )
            self.location_mode = 'static'

    def bin_target_for(self, material, home_pose):
        """home_pose: {'x':..,'y':..,'yaw_deg':..} - the CURRENT confirmed
        home (see TrashSorterPipeline.home_pose), not a static config value.
        Bin offsets are only meaningful relative to a home that was actually
        confirmed this session - /odom re-zeros on every driver restart."""
        offset = self.targets.get(material)
        if offset is None:
            if self.logger:
                self.logger.error(f'PlaceExecutor: no configured bin for material "{material}"')
            return None
        return resolve_offset_target(home_pose['x'], home_pose['y'], home_pose['yaw_deg'], offset)

    def navigate_to_bin(self, material, home_pose, base, imu, running_flag=None):
        """Raise the held object clear of the LiDAR/camera, then navigate to
        the bin for `material` using whichever mode is configured. Returns
        True once an approach is complete, False on failure (bin not
        configured for 'static', or not found for 'detect')."""
        self.arm.move_to_pose('carry')
        time.sleep(2.0)
        self._recentered_joint1 = None

        if self.location_mode == 'detect':
            return self._navigate_to_bin_by_detection(material, running_flag=running_flag)
        return self._navigate_to_bin_static(material, home_pose, base, imu, running_flag=running_flag)

    def _navigate_to_bin_by_detection(self, material, running_flag=None):
        """Rotate in place searching for the bin (VLM), coarse-align the body
        to its centre, then approach with the LiDAR - the same shape as
        SCAN/APPROACH for the trash object itself, just for a bin."""
        detection = self._search_for_bin(material, running_flag=running_flag)
        if detection is None:
            if self.logger:
                self.logger.error(f'PlaceExecutor: could not find "{material}" bin visually after full search')
            return False

        aligned = self._align_to_bin(material, detection, running_flag=running_flag)
        if not aligned and self.logger:
            self.logger.warn('PlaceExecutor: bin alignment did not fully converge - approaching anyway')

        self.movement.drive_until_lidar(self.drop_lidar_distance_m, speed=0.04, running_flag=running_flag)

        # Driving forward can leave the bin slightly off from where the
        # body-level align last saw it (the standoff distance shrinks, so a
        # small heading error grows into a bigger pixel error up close). Do
        # one more fine correction with the arm turret now that the robot
        # has actually stopped - mirrors how the trash object itself gets a
        # body-level coarse_align followed by an arm-level fine_align.
        self._recentered_joint1 = self._recenter_with_arm(material, running_flag=running_flag)
        return True

    def _recenter_with_arm(self, material, running_flag=None):
        """Fine turret-only correction once stopped at the bin. Returns the
        joint1 pulse it ended on (whether or not it fully converged), or
        None if there's no camera/bin_detector to recentre with."""
        if self.camera is None or self.bin_detector is None:
            return None

        current_j1 = float(self.arm.poses.get('carry', {}).get(1, 500))
        for i in range(self.recenter_max_iterations):
            if running_flag is not None and not running_flag():
                return current_j1

            frame = self.camera.get_frame()
            detection = self.bin_detector.detect(frame, material) if frame is not None else None
            if detection is None or not detection.visible:
                if self.logger:
                    self.logger.warn('PlaceExecutor: bin not visible for arm recentre - using last known position')
                return current_j1

            pixel_offset = detection.center_x - self.cam_center_x
            if abs(pixel_offset) <= self.search_pixel_tolerance:
                if self.logger:
                    self.logger.info(f'PlaceExecutor: arm recentre complete, offset={pixel_offset:.0f}px')
                return current_j1

            current_j1 += pixel_offset / self.pixels_per_pulse
            self.arm.send({1: int(round(current_j1))}, duration=0.3)
            if self.logger:
                self.logger.info(
                    f'PlaceExecutor: arm recentre offset={pixel_offset:.0f}px -> joint1={current_j1:.0f} '
                    f'(iter {i + 1}/{self.recenter_max_iterations})'
                )
            time.sleep(0.4)

        if self.logger:
            self.logger.warn('PlaceExecutor: arm recentre did not fully converge after max iterations')
        return current_j1

    def _search_for_bin(self, material, running_flag=None):
        """Rotate step-by-step until the bin becomes visible. Returns the
        accepting Detection, or None if the search ran out of steps."""
        for step in range(self.search_max_steps):
            if running_flag is not None and not running_flag():
                return None

            frame = self.camera.get_frame()
            if frame is not None:
                detection = self.bin_detector.detect(frame, material)
                if detection.visible:
                    if self.logger:
                        self.logger.info(f'PlaceExecutor: "{material}" bin visible at x={detection.center_x}')
                    return detection

            if self.logger:
                self.logger.info(f'PlaceExecutor: bin not visible, search step {step + 1}/{self.search_max_steps}')
            self.movement.rotate_by_angle(self.search_step_deg, running_flag=running_flag)
            time.sleep(self.search_pause_s)
        return None

    def _align_to_bin(self, material, detection, running_flag=None):
        """Rotate the body until the bin's centre is within
        search_pixel_tolerance, re-detecting after every rotation - a single
        unverified rotate-then-drive could leave the robot off-centre if that
        one reading was noisy or the rotation under/overshot.

        VLM bin detection is noisier than the trained object detector - a bad
        read can report a wildly different pixel than the previous one (seen
        in practice: -170px then +195px in consecutive iterations, on the
        same physical bin). Without an outlier check, a single bad frame like
        that sends the rotation the wrong way and the alignment loses the bin
        entirely. Reject an implausible jump instead of trusting it, the same
        way navigation.alignment.Alignment.fine_align does for the object.
        """
        last_center_x = detection.center_x if (detection is not None and detection.visible) else None
        misses = 0
        for step in range(self.search_align_max_steps):
            if running_flag is not None and not running_flag():
                return False

            if detection is None or not detection.visible:
                frame = self.camera.get_frame()
                detection = self.bin_detector.detect(frame, material) if frame is not None else None

            if detection is None or not detection.visible:
                misses += 1
                if misses > self.search_align_max_steps:
                    if self.logger:
                        self.logger.warn('PlaceExecutor: lost the bin while aligning')
                    return False
                detection = None
                time.sleep(0.3)
                continue

            if last_center_x is not None and abs(detection.center_x - last_center_x) > self.search_outlier_px:
                if self.logger:
                    self.logger.warn(
                        f'PlaceExecutor: rejecting bin jump from x={last_center_x} to x={detection.center_x}'
                    )
                misses += 1
                detection = None
                if misses > self.search_align_max_steps:
                    if self.logger:
                        self.logger.warn('PlaceExecutor: lost the bin while aligning')
                    return False
                time.sleep(0.3)
                continue

            misses = 0
            last_center_x = detection.center_x
            pixel_offset = detection.center_x - self.cam_center_x
            if abs(pixel_offset) <= self.search_pixel_tolerance:
                if self.logger:
                    self.logger.info(f'PlaceExecutor: bin align complete, offset={pixel_offset:.0f}px')
                return True

            angle_deg = pixel_offset_to_angle_deg(pixel_offset, self.cam_focal_x)
            if self.logger:
                self.logger.info(
                    f'PlaceExecutor: bin offset={pixel_offset:.0f}px angle={angle_deg:.1f}deg '
                    f'(align iter {step + 1}/{self.search_align_max_steps})'
                )
            self.movement.rotate_by_angle(angle_deg, running_flag=running_flag)
            time.sleep(0.3)
            detection = None  # force a fresh re-detect next iteration

        if self.logger:
            self.logger.warn('PlaceExecutor: bin alignment exceeded max iterations without converging')
        return False

    def _navigate_to_bin_static(self, material, home_pose, base, imu, running_flag=None):
        """Drive to the calibrated home+offset odom coordinate.

        The configured bin offset is the intended stop point. Once odometry has
        reached it we stop there and do NOT run any further, forward-only LiDAR
        approach: that extra drive, started from a position where the small bin
        has already dropped out of the 2D LiDAR scan plane (so drive_until_lidar
        only ever reads the far wall and never sees a close target to stop on),
        is what carried the robot past its manually-set bin position.

        A guarded LiDAR nudge is only attempted when odometry did not converge,
        and it is cut off the moment odometry shows the robot has arrived at (or
        otherwise moved past) the configured bin coordinate, so it can never
        drive the robot onward past the bin either.
        """
        target = self.bin_target_for(material, home_pose)
        if target is None:
            return False

        reached = self.position.go_to_pose(base, imu, *target, running_flag=running_flag)
        if reached:
            if self.logger:
                self.logger.info('PlaceExecutor: odom nav reached the configured bin coordinate - no further approach needed')
            return True

        if self.logger:
            self.logger.warn('PlaceExecutor: odom nav to the bin did not converge - attempting a guarded LiDAR approach')

        distance = self.movement.lidar.front_distance() if self.movement.lidar.is_ready() else None
        if distance is None:
            if self.logger:
                self.logger.warn('PlaceExecutor: no LiDAR reading for a guarded approach - dropping at current stop')
            return True
        if distance <= self.drop_lidar_distance_m + 0.02:
            if self.logger:
                self.logger.warn('PlaceExecutor: LiDAR already reads close - not driving further')
            return True

        # Compose the caller's running_flag with an overrun guard that latches
        # as soon as odometry shows we have arrived at (or overshot) the bin.
        # Once latched, running_flag() returns False and drive_until_lidar's
        # loop exits (stopping the base), so the robot never drives past the
        # configured bin position even if it never sees the bin on the LiDAR.
        target_x, target_y, _ = target
        arrived_at_bin = {'done': False}

        def _guarded_running():
            if running_flag is not None and not running_flag():
                return False
            pose = self.position.current_pose()
            if pose is None:
                return True
            cur_dist = math.hypot(target_x - pose[0], target_y - pose[1])
            if cur_dist < self.drop_overrun_margin_m:
                arrived_at_bin['done'] = True
                if self.logger:
                    self.logger.info(
                        f'PlaceExecutor: within {self.drop_overrun_margin_m:.3f}m of the configured '
                        'bin coordinate - stopping guarded approach'
                    )
            return not arrived_at_bin['done']

        self.movement.drive_until_lidar(self.drop_lidar_distance_m, speed=0.04, running_flag=_guarded_running)
        return True

    def drop(self):
        # Use the recentred turret angle from navigate_to_bin's arm recentre
        # if there is one - move_to_pose('drop') would otherwise snap
        # joint1 back to the drop pose's fixed value and undo it.
        pose = dict(self.arm.poses.get('drop', {}))
        if self._recentered_joint1 is not None:
            pose[1] = self._recentered_joint1
        self.arm.send(pose)
        self._recentered_joint1 = None

        time.sleep(2.0)
        self.gripper.open()
        time.sleep(1.0)
        self.arm.go_home()
        time.sleep(2.0)
