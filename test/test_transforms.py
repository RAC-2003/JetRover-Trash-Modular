from trash_modular.utils.transforms import (
    angle_to_target_deg,
    normalize_deg,
    pixel_offset_to_angle_deg,
)


def test_normalize_deg_wraps_positive():
    assert normalize_deg(190) == -170


def test_normalize_deg_wraps_negative():
    assert normalize_deg(-190) == 170


def test_normalize_deg_identity_within_range():
    assert normalize_deg(45) == 45


def test_angle_to_target_deg_straight_ahead():
    assert angle_to_target_deg(1.0, 0.0) == 0.0


def test_angle_to_target_deg_left():
    assert angle_to_target_deg(0.0, 1.0) == 90.0


def test_pixel_offset_to_angle_deg_center_is_zero():
    assert pixel_offset_to_angle_deg(0.0, 400.0) == 0.0


def test_pixel_offset_to_angle_deg_sign_matches_offset():
    assert pixel_offset_to_angle_deg(100.0, 400.0) > 0
    assert pixel_offset_to_angle_deg(-100.0, 400.0) < 0
