"""Full pipeline orchestrator. This file only wires modules together and
steps the state machine - all the actual logic (detection, alignment,
grasping, navigation) lives in the modules it calls. Build and test every
other module independently before running this.

Control interface: publish "start", "stop", or "sethome" (std_msgs/String) to
/trash_sorter/command. There is no raw-terminal keyboard listener here (the
old project used tty/termios directly in the node) - that makes this
impossible to drive from a test or script. A convenience keyboard client can
be layered on top by publishing to that same topic.

HOME: /odom re-zeros to wherever the robot physically is when its driver
boots - it is NOT a fixed real-world point. bins.home in config.yaml is only
a fallback default; before running for real, place the robot at the actual
physical home spot and publish "sethome" to confirm it, the same way
scripts/calibrate_home_bin.py's jog prompt does. Every RETURN and every bin
offset (home + offset) is computed from whatever home_pose currently holds,
so an unconfirmed home silently sends the robot to the wrong place.
"""

import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from trash_modular.config.params import load_config
from trash_modular.hardware.arm import Arm
from trash_modular.hardware.base import RobotBase
from trash_modular.hardware.gripper import Gripper
from trash_modular.manipulation.grasp import GraspExecutor
from trash_modular.manipulation.place import PlaceExecutor
from trash_modular.navigation.alignment import Alignment
from trash_modular.navigation.movement import Movement
from trash_modular.navigation.position import PositionTracker
from trash_modular.perception.bin_detector import create_bin_detector
from trash_modular.perception.camera import Camera
from trash_modular.perception.depth import DepthSensor
from trash_modular.perception.imu import ImuSensor
from trash_modular.perception.lidar import Lidar
from trash_modular.perception.object_detector import create_detector
from trash_modular.pipeline.state_machine import State, StateMachine


class TrashSorterPipeline(Node):
    def __init__(self):
        super().__init__('trash_sorter_pipeline')
        self.config = load_config()
        log = self.get_logger()

        # Hardware
        self.base = RobotBase(self, self.config, log)
        self.arm = Arm(self, self.config, log)
        self.gripper = Gripper(self, self.arm, self.config, log)

        # Perception
        self.camera = Camera(self, self.config, log)
        self.depth = DepthSensor(self, self.config, log)
        self.imu = ImuSensor(self, self.config, log)
        self.lidar = Lidar(self, self.config, log)
        self.detector = create_detector(self.config, log)

        # Navigation
        self.movement = Movement(self.base, self.imu, self.lidar, self.config, log)
        self.position = PositionTracker(self, self.config, log)
        self.alignment = Alignment(self.movement, self.arm, self.config, log)

        # Manipulation
        self.grasp_executor = GraspExecutor(self.arm, self.gripper, self.config, log)
        bin_detector = None
        if self.config.get('bins', {}).get('location_mode', 'static') == 'detect':
            bin_detector = create_bin_detector(self.config, log)
        self.place_executor = PlaceExecutor(
            self.arm, self.gripper, self.movement, self.position, self.config, log,
            camera=self.camera, bin_detector=bin_detector,
        )

        # Pipeline config
        det_cfg = self.config.get('detection', {})
        self.confidence_threshold = float(det_cfg.get('confidence_threshold', 0.6))
        pipe_cfg = self.config.get('pipeline', {})
        self.search_step_deg = float(pipe_cfg.get('search_step_deg', 30))
        self.search_pause_s = float(pipe_cfg.get('search_pause_s', 2.0))
        self.grasp_max_attempts = int(pipe_cfg.get('grasp_max_attempts', 1))
        self.allow_depth_fallback = bool(pipe_cfg.get('allow_depth_fallback', False))
        self.depth_retry_delay_s = float(pipe_cfg.get('depth_retry_delay_s', 0.5))
        nav_cfg = self.config.get('navigation', {})
        self.approach_lidar_distance = float(nav_cfg.get('approach_lidar_distance_m', 0.140))
        bins_cfg = self.config.get('bins', {})
        self.home_pose = bins_cfg.get('home', {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})
        self._home_confirmed = False

        self.sm = StateMachine(State.IDLE)
        self._run_requested = False
        self._stop_requested = False
        self._active = True
        self._search_steps_done = 0
        self.trial_log = []

        self._detection = None
        self._object_point = None
        self._fine_result = None
        self._grasped = False
        self._material = None
        self._pick_time = None

        self.create_subscription(String, '/trash_sorter/command', self._command_cb, 10)
        log.info('TrashSorterPipeline ready (state=IDLE). Publish "start"/"stop"/"sethome" to /trash_sorter/command')
        log.warn(
            f'home_pose defaults to the config.yaml fallback {self.home_pose} until "sethome" is '
            'published - place the robot at the real home spot and confirm it before starting a cycle'
        )

        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._active = False
        self.base.stop()
        super().destroy_node()

    # ── control ──────────────────────────────────────────────

    def _command_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            if not self._home_confirmed:
                self.get_logger().warn(
                    'Command: START - home was never confirmed with "sethome"; RETURN and bin '
                    f'navigation will target the config.yaml fallback {self.home_pose}, which is only '
                    'correct if the base driver happened to boot with the robot physically there'
                )
            self.get_logger().info('Command: START')
            self._run_requested = True
            self._stop_requested = False
        elif cmd == 'stop':
            self.get_logger().info('Command: STOP')
            self._stop_requested = True
        elif cmd == 'sethome':
            pose = self.position.current_pose()
            if pose is None:
                self.get_logger().error('Command: SETHOME failed - no /odom data yet')
                return
            self.home_pose = {'x': pose[0], 'y': pose[1], 'yaw_deg': pose[2]}
            self._home_confirmed = True
            self.get_logger().info(f'Command: SETHOME - home confirmed at {self.home_pose}')

    def _running_flag(self):
        return self._active and not self._stop_requested

    # ── main loop ────────────────────────────────────────────

    def _run_loop(self):
        handlers = {
            State.IDLE: self._do_idle,
            State.SCAN: self._do_scan,
            State.DETECTED: self._do_detected,
            State.APPROACH: self._do_approach,
            State.ALIGN: self._do_align,
            State.GRASP: self._do_grasp,
            State.TRANSPORT: self._do_transport,
            State.DROP: self._do_drop,
            State.RETURN: self._do_return,
            State.SAFE_STOP: self._do_safe_stop,
        }
        while self._active:
            try:
                handlers[self.sm.state]()
            except Exception as e:
                self.get_logger().error(f'Pipeline error in state {self.sm.state.value}: {e}')
                self.base.stop()
                if self.sm.can_transition(State.SAFE_STOP):
                    self.sm.transition(State.SAFE_STOP)
                time.sleep(0.5)

    def _safe_stop(self, reason):
        self.get_logger().error(f'Safety stop: {reason}')
        self.base.stop()
        if self.sm.can_transition(State.SAFE_STOP):
            self.sm.transition(State.SAFE_STOP)

    # ── states ───────────────────────────────────────────────

    def _do_idle(self):
        if self._run_requested:
            self._run_requested = False
            self._search_steps_done = 0
            # Guarantee the arm is at "home" before searching starts - it's
            # left there after every grasp/drop cycle already, but on a fresh
            # launch (before any cycle has run) nothing else puts it there,
            # and a stray arm pose can occlude the camera during search/align.
            self.get_logger().info('Moving arm to home before starting search')
            self.arm.go_home()
            time.sleep(self.arm.default_duration)
            self.sm.transition(State.SCAN)
        else:
            time.sleep(0.3)

    def _do_scan(self):
        if self._stop_requested:
            self.base.stop()
            self.sm.transition(State.IDLE) if self.sm.can_transition(State.IDLE) else self._safe_stop('stop requested')
            self._stop_requested = False
            return
        if not self.camera.is_ready():
            self._safe_stop('camera unavailable')
            return

        frame = self.camera.get_frame()
        detection = self.detector.detect(frame)
        if detection.visible and detection.confidence >= self.confidence_threshold:
            self.get_logger().info(
                f'Detected {detection.material} conf={detection.confidence:.2f} '
                f'pixel=({detection.center_x},{detection.center_y})'
            )
            self._detection = detection
            self.base.stop()
            self.sm.transition(State.DETECTED)
            return

        self.movement.rotate_by_angle(self.search_step_deg, running_flag=self._running_flag)
        time.sleep(self.search_pause_s)

    def _do_detected(self):
        if not self.depth.is_ready() and not self.allow_depth_fallback:
            self.get_logger().warn('Depth unavailable and fallback disabled - returning to SCAN')
            self.sm.transition(State.SCAN)
            return

        point = self.depth.pixel_to_point(self._detection.center_x, self._detection.center_y)
        if point is None:
            # A single frame having no valid depth at this pixel doesn't mean
            # depth is unavailable there - depth cameras commonly drop single
            # frames, and round/reflective objects (an apple) can lose the
            # IR return on part of their surface for a frame or two. Retry
            # once against a fresh frame before giving up, same as the old
            # project's detection-lock retry, instead of bouncing straight
            # back to SCAN and hitting the identical failure instantly.
            self.get_logger().warn('No depth reading for detection - retrying once')
            time.sleep(self.depth_retry_delay_s)
            point = self.depth.pixel_to_point(self._detection.center_x, self._detection.center_y)
        if point is None:
            self.get_logger().warn('No depth reading after retry - returning to SCAN')
            self.sm.transition(State.SCAN)
            return

        self._object_point = point
        self._pick_time = datetime.now()
        self.sm.transition(State.APPROACH)

    def _do_approach(self):
        aligned, detection = self.alignment.coarse_align(
            self.camera, self.detector, self.confidence_threshold, running_flag=self._running_flag
        )
        if not aligned:
            self.get_logger().warn('Coarse alignment failed - returning to SCAN')
            self.sm.transition(State.SCAN)
            return
        self._detection = detection

        reached = self.movement.drive_until_lidar(self.approach_lidar_distance, running_flag=self._running_flag)
        if not reached:
            self.get_logger().warn('Approach did not reach target standoff distance - continuing cautiously')
        self.sm.transition(State.ALIGN)

    def _do_align(self):
        start_j1 = self.arm.poses.get('home', {}).get(1, 500)

        # self._detection was captured before drive_until_lidar moved the
        # robot in APPROACH - its pixel is stale by now (often a large shift,
        # e.g. after closing from ~0.4m to ~0.12m standoff), so seeding
        # fine_align with it looks like an outlier jump against the first
        # real post-drive detection and wastes misses recovering from that
        # instead of converging. Re-detect now, right before aligning.
        frame = self.camera.get_frame()
        fresh = self.detector.detect(frame) if frame is not None else None
        if fresh is not None and fresh.visible and fresh.confidence >= self.confidence_threshold:
            seed_x, seed_y = fresh.center_x, fresh.center_y
        else:
            self.get_logger().warn('No fresh detection before fine-align - seeding with the pre-approach pixel')
            seed_x, seed_y = self._detection.center_x, self._detection.center_y

        result = self.alignment.fine_align(
            self.camera, self.detector, self.confidence_threshold,
            seed_x, seed_y,
            start_j1, running_flag=self._running_flag,
        )
        self._fine_result = result

        if result.final_px is not None:
            point = self.depth.pixel_to_point(result.final_px, result.final_py)
            if point is not None:
                self._object_point = point

        if not result.complete:
            self.get_logger().warn('Fine alignment did not converge - grasp will be attempted and verified')
        self.sm.transition(State.GRASP)

    def _do_grasp(self):
        px, py = self._fine_result.final_px, self._fine_result.final_py
        z = self._object_point[2] if self._object_point else self.approach_lidar_distance

        grasped = False
        for attempt in range(1, self.grasp_max_attempts + 1):
            grasped, deficit = self.grasp_executor.execute(
                px, py, z, self._fine_result.final_joint1_pulse, y_error=self._fine_result.final_y_error
            )
            self.get_logger().info(f'Grasp attempt {attempt}/{self.grasp_max_attempts}: grasped={grasped} deficit={deficit}')
            if grasped:
                break

        self._grasped = grasped
        self.sm.transition(State.TRANSPORT)

    def _do_transport(self):
        self._material = self._detection.material or 'non-recyclable'
        delivered = self.place_executor.navigate_to_bin(
            self._material, self.home_pose, self.base, self.imu, running_flag=self._running_flag
        )
        if not delivered:
            self._safe_stop(f'no bin configured for material "{self._material}"')
            return
        self.sm.transition(State.DROP)

    def _do_drop(self):
        self.place_executor.drop()
        self._log_trial()
        self.sm.transition(State.RETURN)

    def _do_return(self):
        self.position.go_to_pose(
            self.base, self.imu,
            self.home_pose['x'], self.home_pose['y'], self.home_pose['yaw_deg'],
            running_flag=self._running_flag,
        )
        self._detection = None
        self._object_point = None
        self._fine_result = None
        self._grasped = False
        self._material = None

        if self._stop_requested:
            self._stop_requested = False
            self.sm.transition(State.IDLE)
        else:
            self.sm.transition(State.SCAN)

    def _do_safe_stop(self):
        self.base.stop()
        time.sleep(0.5)
        # Operator must explicitly re-issue "start" from IDLE after a safety stop.
        self._run_requested = False
        self.sm.transition(State.IDLE)

    def _log_trial(self):
        entry = {
            'trial': len(self.trial_log) + 1,
            'material': self._material,
            'grasped': self._grasped,
            'timestamp': datetime.now().strftime('%H:%M:%S'),
        }
        self.trial_log.append(entry)
        self.get_logger().info(f'TRIAL: {entry}')


def main(args=None):
    rclpy.init(args=args)
    node = TrashSorterPipeline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
