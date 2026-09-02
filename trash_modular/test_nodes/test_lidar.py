"""ros2 run trash_modular test_lidar

TEST START -> subscribe to /scan -> wait for a reading -> report the front-cone
minimum distance -> PASS/FAIL
"""

import sys

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.perception.lidar import Lidar
from trash_modular.test_nodes._common import banner, result, spin_until


def main(args=None):
    rclpy.init(args=args)
    node = Node('test_lidar')
    log = node.get_logger()
    banner(log, 'TEST START: lidar')

    config = load_config()
    lidar = Lidar(node, config, log)
    log.info('Waiting up to 5.0s for a scan...')

    ready = spin_until(node, lidar.is_ready, 5.0)
    passed = False
    if ready:
        d = lidar.front_distance()
        log.info(f'Front-cone ({lidar.front_cone_deg}deg) minimum distance: {d:.3f}m')
        passed = d is not None
    else:
        log.error('No LiDAR data received - check "ros2 topic hz <lidar.scan_topic>"')

    result(log, passed, 'front distance reported' if passed else 'no scan data')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
