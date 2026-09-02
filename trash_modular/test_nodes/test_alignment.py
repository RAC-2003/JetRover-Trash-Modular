"""ros2 run trash_modular test_alignment -- --coarse
ros2 run trash_modular test_alignment -- --fine
ros2 run trash_modular test_alignment            (no flag: reports readiness + one detection, never moves)

TEST START -> construct camera/detector/movement/arm/alignment -> (optionally)
run one alignment stage -> PASS/FAIL

Safety: with no flag, this never rotates the body or moves the arm.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.hardware.arm import Arm
from trash_modular.hardware.base import RobotBase
from trash_modular.navigation.alignment import Alignment
from trash_modular.navigation.movement import Movement
from trash_modular.perception.camera import Camera
from trash_modular.perception.imu import ImuSensor
from trash_modular.perception.lidar import Lidar
from trash_modular.perception.object_detector import create_detector
from trash_modular.test_nodes._common import banner, result, run_blocking, spin_until


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--coarse', action='store_true')
    parser.add_argument('--fine', action='store_true')
    ns, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = Node('test_alignment')
    log = node.get_logger()
    banner(log, 'TEST START: alignment')

    config = load_config()
    camera = Camera(node, config, log)
    base = RobotBase(node, config, log)
    imu = ImuSensor(node, config, log)
    lidar = Lidar(node, config, log)
    arm = Arm(node, config, log)
    movement = Movement(base, imu, lidar, config, log)
    alignment = Alignment(movement, arm, config, log)
    detector = create_detector(config, log)
    confidence_threshold = config.get('detection', {}).get('confidence_threshold', 0.6)

    if not spin_until(node, camera.is_ready, 5.0):
        result(log, False, 'no camera frame available')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    frame = camera.get_frame()
    detection = detector.detect(frame)
    log.info(f'Initial detection: {detection}')

    if not ns.coarse and not ns.fine:
        result(log, True, 'no --coarse/--fine given - reported detection only, nothing moved')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    if not detection.visible or detection.confidence < confidence_threshold:
        result(log, False, 'no confident detection to align on')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if ns.coarse:
        aligned, final = run_blocking(node, alignment.coarse_align, camera, detector, confidence_threshold, 6, lambda: True)
        result(log, aligned, f'coarse align -> {final}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if aligned else 1)

    start_j1 = arm.poses.get('home', {}).get(1, 500)
    fine = run_blocking(
        node, alignment.fine_align, camera, detector, confidence_threshold,
        detection.center_x, detection.center_y, start_j1, lambda: True,
    )
    result(log, fine.complete, f'fine align -> {fine}')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if fine.complete else 1)


if __name__ == '__main__':
    main()
