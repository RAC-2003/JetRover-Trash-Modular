"""ros2 run trash_modular test_arm -- --pose home
ros2 run trash_modular test_arm -- --pose carry
ros2 run trash_modular test_arm            (no flag: reports configured poses, moves nothing)

TEST START -> construct Arm -> (optionally) send one named pose -> PASS/FAIL

Safety: with no --pose flag, this never commands the arm.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.hardware.arm import Arm
from trash_modular.test_nodes._common import banner, result


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--pose', default=None, help='Named pose to move to (see config.yaml arm.poses)')
    ns, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = Node('test_arm')
    log = node.get_logger()
    banner(log, 'TEST START: arm')

    config = load_config()
    arm = Arm(node, config, log)
    log.info(f'Configured poses: {list(arm.poses)}')

    if ns.pose is None:
        result(log, True, 'arm constructed, no --pose given so nothing was moved')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    log.info(f'Moving to pose "{ns.pose}" in 2s - Ctrl+C now to abort...')
    time.sleep(2.0)
    ok = arm.move_to_pose(ns.pose)
    time.sleep(2.0)

    result(log, ok, f'sent pose "{ns.pose}"' if ok else f'unknown pose "{ns.pose}"')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
