"""ros2 run trash_modular test_imu

TEST START -> subscribe to IMU raw -> wait for messages -> report integrated
heading over a short window -> PASS/FAIL
"""

import sys
import time

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.perception.imu import ImuSensor
from trash_modular.test_nodes._common import banner, result, spin_until


def main(args=None):
    rclpy.init(args=args)
    node = Node('test_imu')
    log = node.get_logger()
    banner(log, 'TEST START: imu')

    config = load_config()
    imu = ImuSensor(node, config, log)
    log.info('Waiting up to 5.0s for the first IMU message...')

    ready = spin_until(node, imu.is_ready, 5.0)
    passed = False
    if ready:
        start_yaw = imu.integrated_yaw_deg()
        log.info(f'IMU active. Initial integrated yaw={start_yaw:.2f}deg. Sampling for 2s (keep the robot still)...')
        end = time.time() + 2.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        end_yaw = imu.integrated_yaw_deg()
        drift = abs(end_yaw - start_yaw)
        log.info(f'Yaw after 2s still: {end_yaw:.2f}deg (drift={drift:.2f}deg)')
        passed = True
    else:
        log.error('No IMU data received - check "ros2 topic hz <imu.raw_topic>"')

    result(log, passed, 'IMU heading integrating' if passed else 'no IMU data')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
