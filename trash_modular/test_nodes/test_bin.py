"""ros2 run trash_modular test_bin -- --material recyclable
ros2 run trash_modular test_bin -- --material non-recyclable

TEST START -> jog to HOME -> raise arm to carry -> find the bin (static
offset or visual search, whichever config.yaml bins.location_mode is set to)
-> approach with the LiDAR -> simulate a drop (arm to drop pose, open
gripper, arm home) -> PASS/FAIL

Exercises manipulation.place.PlaceExecutor in isolation - no need to run a
full search->grasp cycle first, and no object needs to actually be in the
gripper (this only tests the approach/drop motion, not a real pick-and-place).
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.hardware.arm import Arm
from trash_modular.hardware.base import RobotBase
from trash_modular.hardware.gripper import Gripper
from trash_modular.manipulation.place import PlaceExecutor
from trash_modular.navigation.movement import Movement
from trash_modular.navigation.position import PositionTracker
from trash_modular.perception.imu import ImuSensor
from trash_modular.perception.lidar import Lidar
from trash_modular.scripts.calibrate_home_bin import jog_until_enter
from trash_modular.test_nodes._common import banner, result, run_blocking


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--material', default='recyclable', choices=['recyclable', 'non-recyclable'])
    ns, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = Node('test_bin')
    log = node.get_logger()
    banner(log, f'TEST START: bin ({ns.material})')

    config = load_config()
    location_mode = config.get('bins', {}).get('location_mode', 'static')
    log.info(f'bins.location_mode = "{location_mode}"')

    base = RobotBase(node, config, log)
    imu = ImuSensor(node, config, log)
    lidar = Lidar(node, config, log)
    movement = Movement(base, imu, lidar, config, log)
    position = PositionTracker(node, config, log)
    arm = Arm(node, config, log)
    gripper = Gripper(node, arm, config, log)

    camera, bin_detector = None, None
    if location_mode == 'detect':
        from trash_modular.perception.bin_detector import create_bin_detector
        from trash_modular.perception.camera import Camera
        camera = Camera(node, config, log)
        bin_detector = create_bin_detector(config, log)

    place_executor = PlaceExecutor(
        arm, gripper, movement, position, config, log, camera=camera, bin_detector=bin_detector
    )

    jog_until_enter(node, base, position, '\nJog the robot to HOME, then press ENTER to start the bin-approach test...')
    home = position.current_pose()
    if home is None:
        result(log, False, 'no /odom data - cannot proceed')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    home_pose = {'x': home[0], 'y': home[1], 'yaw_deg': home[2]}
    log.info(f'HOME captured: {home_pose}')

    log.info(f'Finding and approaching the "{ns.material}" bin...')
    approached = run_blocking(
        node, place_executor.navigate_to_bin, ns.material, home_pose, base, imu, lambda: True
    )

    if not approached:
        result(log, False, 'failed to find/approach the bin - see the nav/search log above')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    log.info('Approach complete - simulating a drop (no object required)...')
    run_blocking(node, place_executor.drop)

    result(log, True, 'bin approach + simulated drop completed')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
