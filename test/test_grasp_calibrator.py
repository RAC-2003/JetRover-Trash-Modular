import csv
import os

from trash_modular.manipulation.grasp import GraspCalibrator


def _write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['px', 'py', 'z3d', 'delta_j1', 'delta_lift', 'delta_reach'])
        writer.writerows(rows)


def test_missing_csv_always_falls_back_to_baseline(tmp_path):
    calibrator = GraspCalibrator(str(tmp_path / 'missing.csv'))
    j1, lift, reach, used = calibrator.predict(300, 200, 0.15, baseline_j1=500, baseline_lift=350, baseline_reach=215)
    assert (j1, lift, reach, used) == (500, 350, 215, False)


def test_far_sample_falls_back_to_baseline(tmp_path):
    csv_path = tmp_path / 'calib.csv'
    _write_csv(csv_path, [[1000, 1000, 5.0, 50, 50, 50]])  # far in pixel+depth space
    calibrator = GraspCalibrator(str(csv_path), max_trusted_distance_px=220.0)
    j1, lift, reach, used = calibrator.predict(300, 200, 0.15, baseline_j1=500, baseline_lift=350, baseline_reach=215)
    assert used is False
    assert (j1, lift, reach) == (500, 350, 215)


def test_close_sample_applies_correction(tmp_path):
    csv_path = tmp_path / 'calib.csv'
    _write_csv(csv_path, [[300, 200, 0.15, 10, -5, 3]])
    calibrator = GraspCalibrator(str(csv_path), max_trusted_distance_px=220.0)
    j1, lift, reach, used = calibrator.predict(302, 201, 0.151, baseline_j1=500, baseline_lift=350, baseline_reach=215)
    assert used is True
    assert j1 == 510
    assert lift == 345
    assert reach == 218


def test_missing_pixel_or_depth_falls_back(tmp_path):
    csv_path = tmp_path / 'calib.csv'
    _write_csv(csv_path, [[300, 200, 0.15, 10, -5, 3]])
    calibrator = GraspCalibrator(str(csv_path))
    j1, lift, reach, used = calibrator.predict(None, None, None, baseline_j1=500, baseline_lift=350, baseline_reach=215)
    assert used is False
