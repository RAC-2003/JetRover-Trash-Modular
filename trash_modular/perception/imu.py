"""IMU heading. Wheel odometry yaw is unreliable during in-place spins on this
base (wheel slip), so navigation uses gyro-integrated heading from here
instead - ported from trash_sorter.py's imu_raw_integrated_yaw. gyro_sign is
config because this robot's gyro reports z opposite the ROS convention
(verified empirically in the old project); a different unit may not need it.
"""

import math
import time

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from trash_modular.utils.transforms import normalize_deg


class ImuSensor:
    def __init__(self, node, config, logger=None):
        self.node = node
        self.logger = logger or node.get_logger()

        imu_cfg = config.get('imu', {})
        topic = imu_cfg.get('raw_topic', '/ros_robot_controller/imu_raw')
        self.gyro_sign = float(imu_cfg.get('gyro_sign', -1.0))
        self.ready_timeout_s = float(imu_cfg.get('ready_timeout_s', 2.0))

        self._integrated_yaw_deg = 0.0
        self._last_msg_time = None
        self._last_wall_time = 0.0

        node.create_subscription(Imu, topic, self._callback, qos_profile_sensor_data)
        self.logger.info(f'ImuSensor ready - subscribed to {topic}, gyro_sign={self.gyro_sign}')

    def _callback(self, msg):
        now = self.node.get_clock().now()
        if self._last_msg_time is not None:
            dt = (now - self._last_msg_time).nanoseconds / 1e9
            if 0 < dt < 0.5:
                self._integrated_yaw_deg += math.degrees(self.gyro_sign * msg.angular_velocity.z * dt)
        self._last_msg_time = now
        self._last_wall_time = time.time()

    def is_ready(self):
        if self._last_msg_time is None:
            return False
        return (time.time() - self._last_wall_time) <= self.ready_timeout_s

    def integrated_yaw_deg(self):
        return normalize_deg(self._integrated_yaw_deg)

    def raw_integrated_yaw_deg(self):
        """Unwrapped cumulative yaw, used as a delta reference by navigation."""
        return self._integrated_yaw_deg
