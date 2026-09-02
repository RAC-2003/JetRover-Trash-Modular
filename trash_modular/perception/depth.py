"""Depth access + pixel -> 3D point conversion.

Reads live CameraInfo intrinsics instead of the hardcoded focal
length/center constants the old trash_sorter.py used - those go stale the
moment the camera is swapped or re-calibrated.

The old project ran this as a separate node (trash_position.py) that talked
to the detector node over a hand-parsed std_msgs/String topic
("TRASH DETECTED: ... center=(x,y) ..."). Here it's just a class the
perception/pipeline code calls directly with a pixel - same process, no
string protocol, still independently testable via test_depth.
"""

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


class DepthSensor:
    def __init__(self, node, config, logger=None):
        self.node = node
        self.logger = logger or node.get_logger()
        self.bridge = CvBridge()

        cam_cfg = config.get('camera', {})
        depth_topic = cam_cfg.get('depth_topic', '/depth_cam/depth/image_raw')
        info_topic = cam_cfg.get('depth_info_topic', '/depth_cam/depth/camera_info')

        self._depth_image = None
        self._encoding = None
        self._camera_info = None

        node.create_subscription(Image, depth_topic, self._depth_cb, 10)
        node.create_subscription(CameraInfo, info_topic, self._info_cb, 10)
        self.logger.info(f'DepthSensor ready - {depth_topic}, {info_topic}')

    def _depth_cb(self, msg):
        try:
            self._encoding = msg.encoding
            self._depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.logger.error(f'DepthSensor: failed to decode depth image: {e}')

    def _info_cb(self, msg):
        self._camera_info = msg

    def is_ready(self):
        return self._depth_image is not None and self._camera_info is not None

    def pixel_to_point(self, px, py, region=10):
        """Returns (x, y, z) in metres in the depth camera frame, or None."""
        if not self.is_ready():
            self.logger.warn('DepthSensor: depth image or camera info not received yet')
            return None

        h, w = self._depth_image.shape[:2]
        if not (0 <= px < w and 0 <= py < h):
            self.logger.warn(f'DepthSensor: pixel ({px},{py}) out of bounds ({w}x{h})')
            return None

        x1, x2 = max(0, px - region), min(w, px + region)
        y1, y2 = max(0, py - region), min(h, py + region)
        patch = self._depth_image[y1:y2, x1:x2].astype(float)

        if '32F' in (self._encoding or ''):
            valid = patch[(patch > 0.05) & (patch < 5.0)]
            if valid.size == 0:
                self.logger.warn(f'DepthSensor: no valid depth (32F) near ({px},{py})')
                return None
            depth_m = float(np.median(valid))
        else:
            valid = patch[(patch > 50) & (patch < 5000)]
            if valid.size == 0:
                self.logger.warn(f'DepthSensor: no valid depth (16U) near ({px},{py})')
                return None
            depth_m = float(np.median(valid)) / 1000.0

        fx, fy = self._camera_info.k[0], self._camera_info.k[4]
        cx, cy = self._camera_info.k[2], self._camera_info.k[5]

        x = (px - cx) * depth_m / fx
        y = (py - cy) * depth_m / fy
        z = depth_m
        return x, y, z
