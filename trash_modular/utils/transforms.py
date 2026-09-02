"""Single shared implementation of the angle/pose math that was copy-pasted
across trash_sorter.py, V5.py and home_to_bin.py in the old project."""

import math


def quaternion_to_yaw_deg(orientation):
    """orientation: geometry_msgs/Quaternion. Returns yaw in degrees."""
    q = orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def normalize_deg(angle):
    """Wrap an angle in degrees to (-180, 180]."""
    angle = angle % 360.0
    if angle > 180.0:
        angle -= 360.0
    return angle


def angle_to_target_deg(dx, dy):
    return math.degrees(math.atan2(dy, dx))


def pixel_offset_to_angle_deg(pixel_offset, focal_length_px):
    """Convert a horizontal pixel error into a body-rotation angle."""
    return math.degrees(math.atan2(pixel_offset, focal_length_px))
