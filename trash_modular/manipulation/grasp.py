"""Grasp execution: GraspCalibrator (ported from the old project, cleaned up)
plus the sequence that opens the gripper, lowers the arm to a
calibration-corrected pose, closes the gripper, and raises it.

GraspCalibrator corrects the fixed lift/reach arm pose using nearby empirical
samples (grasp_calibration.csv) because the camera and gripper don't share an
optical centre - a pixel that looks centred to the camera doesn't map to a
fixed arm pose once depth varies. It refuses to extrapolate past
max_trusted_distance_px and falls back to the baseline pose instead of
guessing; collect more samples with scripts spanning the pixel/depth range
you actually grasp at to make it more useful.
"""

import csv
import math
import os

from trash_modular.config.params import resolve_path


class GraspCalibrator:
    def __init__(self, csv_path, max_trusted_distance_px=220.0, max_neighbours=4, logger=None):
        self.max_trusted_distance_px = max_trusted_distance_px
        self.max_neighbours = max_neighbours
        self.logger = logger
        self.points = []
        self._load(csv_path)

    def _log(self, msg, level='info'):
        if self.logger:
            getattr(self.logger, level)(msg)

    def _load(self, csv_path):
        if not os.path.isfile(csv_path):
            self._log(f'GraspCalibrator: no calibration file at {csv_path} - using baseline pose always', 'warn')
            return
        with open(csv_path, newline='') as f:
            for row in csv.DictReader(f):
                try:
                    self.points.append({k: float(row[k]) for k in
                                         ('px', 'py', 'z3d', 'delta_j1', 'delta_lift', 'delta_reach')})
                except (KeyError, ValueError) as e:
                    self._log(f'GraspCalibrator: skipping malformed row {row}: {e}', 'warn')
        self._log(f'GraspCalibrator: loaded {len(self.points)} calibration point(s) from {csv_path}')

    DEPTH_SCALE_PX_PER_M = 800.0

    def _distance(self, px, py, z3d, point):
        dpx, dpy = px - point['px'], py - point['py']
        dz = (z3d - point['z3d']) * self.DEPTH_SCALE_PX_PER_M
        return math.sqrt(dpx * dpx + dpy * dpy + dz * dz)

    def predict(self, px, py, z3d, baseline_j1, baseline_lift, baseline_reach):
        """Returns (j1, lift, reach, used_calibration: bool)."""
        if px is None or py is None or z3d is None or not self.points:
            return baseline_j1, baseline_lift, baseline_reach, False

        distances = sorted(((self._distance(px, py, z3d, p), p) for p in self.points), key=lambda d: d[0])
        nearest_dist = distances[0][0]
        if nearest_dist > self.max_trusted_distance_px:
            self._log(
                f'GraspCalibrator: nearest sample is {nearest_dist:.0f}px away '
                f'(limit {self.max_trusted_distance_px:.0f}px) - using baseline pose', 'warn'
            )
            return baseline_j1, baseline_lift, baseline_reach, False

        neighbours = distances[:min(self.max_neighbours, len(distances))]
        weights = [1.0 / d if d > 1e-6 else 1e6 for d, _ in neighbours]
        total = sum(weights)

        def blend(key, baseline):
            delta = sum(w * p[key] for w, (_, p) in zip(weights, neighbours)) / total
            return int(round(baseline + delta))

        j1 = blend('delta_j1', baseline_j1)
        lift = blend('delta_lift', baseline_lift)
        reach = blend('delta_reach', baseline_reach)
        self._log(f'GraspCalibrator: {len(neighbours)} neighbour(s), nearest={nearest_dist:.0f}px -> '
                   f'pose=(j1={j1},lift={lift},reach={reach})')
        return j1, lift, reach, True


class GraspExecutor:
    def __init__(self, arm, gripper, config, logger=None):
        self.arm = arm
        self.gripper = gripper
        self.logger = logger

        grasp_cfg = config.get('arm', {}).get('grasp', {})
        self.baseline_lift = float(grasp_cfg.get('baseline_lift', 350))
        self.baseline_reach = float(grasp_cfg.get('baseline_reach', 215))

        align_cfg = config.get('alignment', {})
        self.pixels_per_pulse_lift = float(align_cfg.get('pixels_per_pulse_lift', 3.0))
        self.lift_correction_max_pulse = float(align_cfg.get('lift_correction_max_pulse', 120))

        csv_path = resolve_path(config, grasp_cfg.get('calibration_csv', 'config/grasp_calibration.csv'))
        self.calibrator = GraspCalibrator(
            csv_path,
            max_trusted_distance_px=float(grasp_cfg.get('max_trusted_distance_px', 220.0)),
            max_neighbours=int(grasp_cfg.get('max_neighbours', 4)),
            logger=logger,
        )

    def execute(self, px, py, z3d, joint1_pulse, y_error=None):
        """Open, lower to a calibration-corrected pose, close, raise. Returns
        (grasped: bool, deficit: float or None) from gripper feedback."""
        self.gripper.open()
        import time
        time.sleep(1.0)

        lift_baseline = self.baseline_lift
        if y_error is not None:
            correction = max(-self.lift_correction_max_pulse,
                              min(self.lift_correction_max_pulse, y_error / self.pixels_per_pulse_lift))
            lift_baseline = self.baseline_lift + correction
            if self.logger and abs(correction) > 0.5:
                self.logger.info(f'GraspExecutor: lift nudge {correction:+.0f} from fine-align y_error={y_error:.0f}px')

        j1, lift, reach, calibrated = self.calibrator.predict(
            px, py, z3d,
            baseline_j1=joint1_pulse, baseline_lift=lift_baseline, baseline_reach=self.baseline_reach,
        )
        if self.logger:
            self.logger.info(
                f'GraspExecutor: pose=(j1={j1},lift={lift},reach={reach}) '
                f'({"calibrated" if calibrated else "baseline"})'
            )

        self.arm.send({1: j1, 2: lift, 3: reach, 4: 250, 5: 500}, duration=2.0)
        time.sleep(2.5)
        self.gripper.close()
        time.sleep(1.0)
        self.arm.go_home()
        time.sleep(2.0)

        return self.gripper.is_grasped()
