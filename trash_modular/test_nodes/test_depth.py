"""ros2 run trash_modular test_depth

TEST START -> subscribe to depth image + camera_info -> wait for both ->
sample the centre pixel -> report (x,y,z) -> PASS/FAIL
"""

import sys

import rclpy
from rclpy.node import Node

from trash_modular.config.params import load_config
from trash_modular.perception.depth import DepthSensor
from trash_modular.test_nodes._common import banner, result, spin_until


def main(args=None):
    rclpy.init(args=args)
    node = Node('test_depth')
    log = node.get_logger()
    banner(log, 'TEST START: depth')

    config = load_config()
    depth = DepthSensor(node, config, log)
    log.info('Waiting up to 5.0s for depth image + camera_info...')

    ready = spin_until(node, depth.is_ready, 5.0)
    passed = False
    if ready:
        w = config.get('camera', {}).get('rgb_width', 640)
        h = config.get('camera', {}).get('rgb_height', 360)
        point = depth.pixel_to_point(w // 2, h // 2)
        if point is not None:
            log.info(f'Centre pixel ({w // 2},{h // 2}) -> x={point[0]:.3f} y={point[1]:.3f} z={point[2]:.3f}m')
            passed = True
        else:
            log.error('Depth/camera_info ready but no valid depth at the centre pixel (point the camera at something)')
    else:
        log.error('Depth image or camera_info never arrived - check depth_topic/depth_info_topic')

    result(log, passed, 'valid depth point returned' if passed else 'no valid depth')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
