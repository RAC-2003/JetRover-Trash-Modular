"""ros2 run trash_modular test_pipeline
ros2 run trash_modular test_pipeline -- --start --duration 30

TEST START -> construct the full pipeline node (every module) -> report
readiness of each sensor -> (optionally) run for a bounded window, logging
state transitions -> stop -> PASS/FAIL

This is an integration smoke test, not a substitute for testing each module
independently first. Run test_camera/test_depth/.../test_gripper before this.
"""

import argparse
import sys
import time

import rclpy

from trash_modular.pipeline.trash_sorter_pipeline import TrashSorterPipeline
from trash_modular.test_nodes._common import banner, result


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--start', action='store_true', help='issue a start command and run for --duration seconds')
    parser.add_argument('--duration', type=float, default=20.0)
    ns, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = TrashSorterPipeline()
    log = node.get_logger()
    banner(log, 'TEST START: pipeline (integration)')

    time.sleep(1.0)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    log.info(
        f'Sensors: camera={node.camera.is_ready()} depth={node.depth.is_ready()} '
        f'imu={node.imu.is_ready()} lidar={node.lidar.is_ready()} odom={node.position.is_ready()}'
    )
    log.info(f'Initial state: {node.sm.state.value}')

    if not ns.start:
        result(log, True, 'pipeline constructed, all modules wired - no --start given, nothing ran')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    log.info(f'Issuing start, running for {ns.duration}s...')
    node._run_requested = True
    last_state = node.sm.state
    deadline = time.time() + ns.duration
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.sm.state != last_state:
            log.info(f'State: {last_state.value} -> {node.sm.state.value}')
            last_state = node.sm.state

    log.info('Duration elapsed - issuing stop')
    node._stop_requested = True
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.base.stop()

    result(log, True, f'ran for {ns.duration}s, ended in state {node.sm.state.value}')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
