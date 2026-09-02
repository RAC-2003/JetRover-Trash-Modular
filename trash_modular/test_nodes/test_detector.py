"""ros2 run trash_modular test_detector

TEST START -> wait for a camera frame -> run the configured detection.strategy
-> report the Detection -> PASS/FAIL

Does not move the robot. Requires ANTHROPIC_API_KEY / OPENAI_API_KEY to be set
for the claude/chatgpt/yolo_hybrid(classify) strategies.
"""

import sys

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.perception.camera import Camera
from trash_modular.perception.object_detector import create_detector
from trash_modular.test_nodes._common import banner, result, spin_until


def main(args=None):
    rclpy.init(args=args)
    node = Node('test_detector')
    log = node.get_logger()
    banner(log, 'TEST START: object_detector')

    config = load_config()
    strategy = config.get('detection', {}).get('strategy', 'claude')
    log.info(f'Strategy: {strategy}')

    camera = Camera(node, config, log)
    log.info('Waiting up to 5.0s for a frame...')
    if not spin_until(node, camera.is_ready, 5.0):
        result(log, False, 'no camera frame available')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        detector = create_detector(config, log)
    except Exception as e:
        log.error(f'Failed to construct detector: {e}')
        result(log, False, f'detector construction failed: {e}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    frame = camera.get_frame()
    log.info('Running detection on the current frame...')
    detection = detector.detect(frame)
    log.info(f'Detection: {detection}')

    passed = True  # a confident "not visible" is still a valid, working detector
    if detection.visible:
        log.info(f'Object visible: material={detection.material} confidence={detection.confidence:.2f}')
    else:
        log.info('No object visible in current frame (this is not necessarily a failure)')

    result(log, passed, f'detector returned {detection}')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
