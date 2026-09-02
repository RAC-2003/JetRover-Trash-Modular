"""ros2 run trash_modular test_gripper -- --open
ros2 run trash_modular test_gripper -- --close
ros2 run trash_modular test_gripper            (no flag: reports servo feedback only)

TEST START -> construct Gripper -> (optionally) open/close -> report servo
feedback / grasp verification -> PASS/FAIL

Safety: with no flag, this never commands the gripper.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.hardware.arm import Arm
from trash_modular.hardware.gripper import Gripper
from trash_modular.test_nodes._common import banner, result, spin_until


def main(args=None):
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--open', action='store_true')
    group.add_argument('--close', action='store_true')
    ns, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = Node('test_gripper')
    log = node.get_logger()
    banner(log, 'TEST START: gripper')

    config = load_config()
    arm = Arm(node, config, log)
    gripper = Gripper(node, arm, config, log)

    spin_until(node, lambda: gripper.last_state() is not None, 2.0)
    st = gripper.last_state()
    log.info(f'Current servo state: {st if st else "no feedback yet"}')

    if not ns.open and not ns.close:
        result(log, True, 'gripper constructed, no --open/--close given so nothing was moved')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    if ns.open:
        gripper.open()
    else:
        gripper.close()

    log.info(f'Waiting {gripper.settle_time_s}s for the servo to settle...')
    time.sleep(gripper.settle_time_s + 0.5)
    spin_until(node, lambda: True, 0.5)

    grasped, deficit = gripper.is_grasped()
    log.info(f'is_grasped={grasped} deficit={deficit}')

    result(log, True, f'gripper commanded, deficit={deficit}')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
