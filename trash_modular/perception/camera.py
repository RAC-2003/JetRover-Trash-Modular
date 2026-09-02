"""RGB camera access. Only responsibility: subscribe, decode, hand back frames.
No detection, no movement, no VLM calls happen here."""

import time

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Camera:
    def __init__(self, node, config, logger=None):
        self.node = node
        self.logger = logger or node.get_logger()
        self.bridge = CvBridge()

        cam_cfg = config.get('camera', {})
        topic = cam_cfg.get('rgb_topic', '/depth_cam/rgb/image_raw')
        self.ready_timeout_s = float(cam_cfg.get('ready_timeout_s', 3.0))

        self._frame = None
        self._last_stamp = 0.0
        node.create_subscription(Image, topic, self._callback, 10)
        self.logger.info(f'Camera ready - subscribed to {topic}')

    def _callback(self, msg):
        try:
            self._frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._last_stamp = time.time()
        except Exception as e:
            self.logger.error(f'Camera: failed to decode frame: {e}')

    def get_frame(self):
        """Returns a copy of the latest RGB frame, or None if none received yet."""
        return None if self._frame is None else self._frame.copy()

    def is_ready(self):
        if self._frame is None:
            return False
        return (time.time() - self._last_stamp) <= self.ready_timeout_s
