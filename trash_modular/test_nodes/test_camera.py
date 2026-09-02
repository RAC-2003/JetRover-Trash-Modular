"""ros2 run trash_modular test_camera

TEST START -> subscribe to RGB topic -> wait for a frame -> report shape -> PASS/FAIL
"""

import sys

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.perception.camera import Camera
from trash_modular.test_nodes._common import banner, result, spin_until


def main(args=None):
    rclpy.init(args=args)
    node = Node('test_camera')
    log = node.get_logger()
    banner(log, 'TEST START: camera')

    config = load_config()
    camera = Camera(node, config, log)
    log.info(f'Waiting up to {camera.ready_timeout_s + 3.0:.1f}s for a frame...')

    got_frame = spin_until(node, camera.is_ready, camera.ready_timeout_s + 3.0)
    passed = False
    if got_frame:
        frame = camera.get_frame()
        log.info(f'Frame received: shape={frame.shape}, dtype={frame.dtype}')
        passed = frame is not None and frame.size > 0
    else:
        log.error('No frame received - check "ros2 topic hz <rgb_topic>" and that the camera driver is running')

    result(log, passed, 'frame received and decoded' if passed else 'no frame received')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
