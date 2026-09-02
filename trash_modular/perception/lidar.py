"""Front-cone LiDAR distance. Used by navigation.movement for closed-loop
approach/standoff driving."""

import math
import time

from sensor_msgs.msg import LaserScan


class Lidar:
    def __init__(self, node, config, logger=None):
        self.node = node
        self.logger = logger or node.get_logger()

        lidar_cfg = config.get('lidar', {})
        topic = lidar_cfg.get('scan_topic', '/scan')
        self.front_cone_deg = float(lidar_cfg.get('front_cone_deg', 30.0))
        self.min_valid_range_m = float(lidar_cfg.get('min_valid_range_m', 0.12))
        self.max_valid_range_m = float(lidar_cfg.get('max_valid_range_m', 16.0))
        self.ready_timeout_s = float(lidar_cfg.get('ready_timeout_s', 2.0))

        self._front_distance = None
        self._last_stamp = 0.0
        node.create_subscription(LaserScan, topic, self._callback, 10)
        self.logger.info(f'Lidar ready - subscribed to {topic}, front_cone={self.front_cone_deg}deg')

    def _callback(self, msg):
        cone_rad = math.radians(self.front_cone_deg)
        front_ranges = []
        for i, r in enumerate(msg.ranges):
            if self.min_valid_range_m < r < self.max_valid_range_m:
                angle = msg.angle_min + i * msg.angle_increment
                if abs(angle) < cone_rad:
                    front_ranges.append(r)
        if front_ranges:
            self._front_distance = min(front_ranges)
            self._last_stamp = time.time()

    def is_ready(self):
        if self._front_distance is None:
            return False
        return (time.time() - self._last_stamp) <= self.ready_timeout_s

    def front_distance(self):
        return self._front_distance
