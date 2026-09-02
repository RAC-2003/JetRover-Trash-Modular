"""ros2 run trash_modular test_movement -- --forward
ros2 run trash_modular test_movement -- --backward
ros2 run trash_modular test_movement -- --rotate 30
ros2 run trash_modular test_movement -- --approach
ros2 run trash_modular test_movement            (no flag: reports sensor readiness, never moves)

TEST START -> construct base/imu/lidar/movement -> (optionally) one explicit,
bounded motion -> PASS/FAIL

Safety: with no flag, this never commands the base. Every motion here is
bounded (fixed duration or closed-loop with a timeout) and ends in stop().
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.hardware.base import RobotBase
from trash_modular.navigation.movement import Movement
from trash_modular.perception.imu import ImuSensor
from trash_modular.perception.lidar import Lidar
from trash_modular.test_nodes._common import banner, result, run_blocking, spin_for


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--forward', action='store_true')
    parser.add_argument('--backward', action='store_true')
    parser.add_argument('--rotate', type=float, default=None, help='degrees, signed')
    parser.add_argument('--approach', action='store_true', help='drive_until_lidar to navigation.approach_lidar_distance_m')
    parser.add_argument('--duration', type=float, default=1.0)
    ns, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = Node('test_movement')
    log = node.get_logger()
    banner(log, 'TEST START: movement')

    config = load_config()
    base = RobotBase(node, config, log)
    imu = ImuSensor(node, config, log)
    lidar = Lidar(node, config, log)
    movement = Movement(base, imu, lidar, config, log)

    spin_for(node, 1.0)  # let imu/lidar populate before reporting readiness
    log.info(f'IMU ready={imu.is_ready()}  LiDAR ready={lidar.is_ready()}')

    passed = True
    if ns.forward:
        log.info(f'Forward for {ns.duration}s...')
        run_blocking(node, movement.forward, None, ns.duration, lambda: True)
    elif ns.backward:
        log.info(f'Backward for {ns.duration}s...')
        run_blocking(node, movement.backward, None, ns.duration, lambda: True)
    elif ns.rotate is not None:
        log.info(f'Rotating {ns.rotate}deg...')
        passed = run_blocking(node, movement.rotate_by_angle, ns.rotate, None, lambda: True)
    elif ns.approach:
        target = config.get('navigation', {}).get('approach_lidar_distance_m', 0.14)
        log.info(f'Driving to LiDAR standoff {target}m...')
        passed = run_blocking(node, movement.drive_until_lidar, target, 0.1, 30.0, lambda: True)
    else:
        result(log, True, 'no motion flag given - base/imu/lidar constructed only')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    base.stop()
    result(log, passed, 'motion completed' if passed else 'motion did not converge/complete')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
