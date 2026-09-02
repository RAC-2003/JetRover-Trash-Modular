"""Pixel-based alignment: coarse (body rotation) then fine (arm turret visual
servo). Pure pixel->angle math lives in utils.transforms; this module only
orchestrates camera+detector+movement/arm around that math.

Fine alignment redesigned (not byte-copied) from the old ALIGN2 loop: same
documented behaviour (outlier rejection, miss counting with a single reseed,
vertical error deferred to grasp time since moving the arm mid-alignment
would occlude the camera), reimplemented as one clear, boundable loop.
"""

import time
from dataclasses import dataclass
from typing import Optional

from trash_modular.perception.object_detector import Detection
from trash_modular.utils.transforms import pixel_offset_to_angle_deg


@dataclass
class FineAlignResult:
    complete: bool
    final_joint1_pulse: float
    final_px: Optional[int]
    final_py: Optional[int]
    final_y_error: Optional[int]


class Alignment:
    def __init__(self, movement, arm, config, logger=None):
        self.movement = movement
        self.arm = arm
        self.logger = logger

        cam_cfg = config.get('camera', {})
        self.cam_center_x = float(cam_cfg.get('rgb_center_x', 324.88))
        self.cam_focal_x = float(cam_cfg.get('rgb_focal_x', 357.69))

        align_cfg = config.get('alignment', {})
        self.coarse_tolerance = float(align_cfg.get('coarse_pixel_tolerance', 30))
        self.fine_tolerance_x = float(align_cfg.get('fine_pixel_tolerance', 15))
        self.target_pixel_x = float(align_cfg.get('target_pixel_x', 329))
        self.target_pixel_y = float(align_cfg.get('target_pixel_y', 180))
        self.outlier_px = float(align_cfg.get('fine_outlier_px', 110))
        self.max_misses = int(align_cfg.get('fine_max_misses', 4))
        self.max_iterations = int(align_cfg.get('fine_max_iterations', 10))
        self.reseed_max = int(align_cfg.get('fine_reseed_max', 1))
        self.pixels_per_pulse = float(align_cfg.get('pixels_per_pulse', 2.2))

    def coarse_align(self, camera, detector, confidence_threshold, max_iterations=6, running_flag=None):
        """Rotate the body until the object's pixel x is within tolerance of
        centre. Returns (aligned: bool, last Detection)."""
        detection = Detection.not_visible()
        for i in range(max_iterations):
            if running_flag is not None and not running_flag():
                return False, detection
            frame = camera.get_frame()
            if frame is None:
                time.sleep(0.2)
                continue

            detection = detector.detect(frame)
            if not detection.visible or detection.confidence < confidence_threshold:
                if self.logger:
                    self.logger.warn('Alignment.coarse_align: object not visible')
                return False, detection

            pixel_offset = detection.center_x - self.cam_center_x
            if abs(pixel_offset) <= self.coarse_tolerance:
                if self.logger:
                    self.logger.info(f'Alignment.coarse_align: complete, offset={pixel_offset:.0f}px')
                return True, detection

            angle_deg = pixel_offset_to_angle_deg(pixel_offset, self.cam_focal_x)
            if self.logger:
                self.logger.info(
                    f'Alignment.coarse_align: offset={pixel_offset:.0f}px angle={angle_deg:.1f}deg '
                    f'(iter {i + 1}/{max_iterations})'
                )
            if not self.movement.rotate_by_angle(angle_deg, running_flag=running_flag):
                return False, detection
            time.sleep(0.5)

        if self.logger:
            self.logger.warn('Alignment.coarse_align: exceeded max iterations without converging')
        return False, detection

    def fine_align(self, camera, detector, confidence_threshold, seed_px, seed_py,
                    start_joint1_pulse, running_flag=None):
        """Turret (arm joint1) visual servo toward target_pixel_x/y. Never
        moves the arm on the vertical axis mid-loop - that's deferred to grasp
        time (see manipulation.grasp) so the arm doesn't occlude the camera."""
        current_joint1 = float(start_joint1_pulse)
        anchor_x, anchor_y = seed_px, seed_py
        misses = 0
        reseeds_used = 0
        complete = False
        last_y_error = None

        for i in range(self.max_iterations):
            if running_flag is not None and not running_flag():
                break

            frame = camera.get_frame()
            detection = detector.detect(frame) if frame is not None else Detection.not_visible()

            accepted = False
            if detection.visible and detection.confidence >= confidence_threshold:
                if anchor_x is None:
                    accepted = True
                else:
                    jump = ((detection.center_x - anchor_x) ** 2 + (detection.center_y - anchor_y) ** 2) ** 0.5
                    accepted = jump <= self.outlier_px
                    if not accepted and self.logger:
                        self.logger.warn(f'Alignment.fine_align: rejecting {jump:.0f}px jump (iter {i + 1})')

            if accepted:
                misses = 0
                anchor_x, anchor_y = detection.center_x, detection.center_y
                x_error = anchor_x - self.target_pixel_x
                last_y_error = anchor_y - self.target_pixel_y

                if abs(x_error) <= self.fine_tolerance_x:
                    complete = True
                    if self.logger:
                        self.logger.info(f'Alignment.fine_align: complete, x_error={x_error:.0f}px')
                    break

                current_joint1 += x_error / self.pixels_per_pulse
                self.arm.send({1: int(round(current_joint1))}, duration=0.3)
                if self.logger:
                    self.logger.info(
                        f'Alignment.fine_align: x_error={x_error:.0f}px y_error={last_y_error:.0f}px '
                        f'-> joint1={current_joint1:.0f} (iter {i + 1}/{self.max_iterations})'
                    )
                time.sleep(0.4)
            else:
                misses += 1
                if misses >= self.max_misses:
                    if reseeds_used < self.reseed_max:
                        reseeds_used += 1
                        misses = 0
                        anchor_x, anchor_y = None, None
                        if self.logger:
                            self.logger.warn('Alignment.fine_align: reseeding after too many misses')
                    else:
                        if self.logger:
                            self.logger.warn('Alignment.fine_align: aborting - too many misses after reseed')
                        break

        return FineAlignResult(
            complete=complete,
            final_joint1_pulse=current_joint1,
            final_px=anchor_x if anchor_x is not None else seed_px,
            final_py=anchor_y if anchor_y is not None else seed_py,
            final_y_error=last_y_error,
        )
