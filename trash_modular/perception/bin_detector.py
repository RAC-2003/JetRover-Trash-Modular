"""Bin detection: locate the bin appropriate for a given material, using the
same VLM client/model/credentials configured under vlm: (Claude or ChatGPT),
just with a different prompt than perception.object_detector's trash-item
detection. Selected via config bins.location_mode: 'detect' (the alternative
to 'static' odom-offset navigation in manipulation/place.py).

YOLO is NOT reused here even under detection.strategy: yolo_hybrid - that
model is trained on detection.classes (the trash items), not bins, so it has
no notion of what a bin looks like unless you train and wire in a separate
model. Bin search always goes through the VLM.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class BinDetection:
    visible: bool
    center_x: Optional[int] = None
    width_px: Optional[int] = None

    @staticmethod
    def not_visible():
        return BinDetection(visible=False)


class BinDetector:
    def __init__(self, vlm_client, width, logger=None):
        self.vlm_client = vlm_client
        self.width = width
        self.logger = logger

    def detect(self, frame, material):
        result = self.vlm_client.detect_bin(frame, material, self.width)
        if not result.get('bin_visible'):
            return BinDetection.not_visible()
        return BinDetection(
            visible=True,
            center_x=result.get('bin_center_x'),
            width_px=result.get('bin_width_px'),
        )


def create_bin_detector(config, logger=None):
    vlm_cfg = config.get('vlm', {})
    strategy = config.get('detection', {}).get('strategy', 'claude')
    width = config.get('camera', {}).get('rgb_width', 640)

    if strategy == 'chatgpt':
        from trash_modular.intelligence.vlm import ChatGPTVLMClient
        client = ChatGPTVLMClient(
            vlm_cfg.get('chatgpt_model', 'gpt-4o'),
            vlm_cfg.get('max_tokens_detect', 80),
            vlm_cfg.get('max_tokens_classify', 40),
            logger=logger,
        )
    else:
        # yolo_hybrid falls back to Claude here too - see module docstring.
        from trash_modular.intelligence.vlm import ClaudeVLMClient
        client = ClaudeVLMClient(
            vlm_cfg.get('claude_model', 'claude-opus-4-5'),
            vlm_cfg.get('max_tokens_detect', 80),
            vlm_cfg.get('max_tokens_classify', 40),
            logger=logger,
        )
    return BinDetector(client, width, logger=logger)
