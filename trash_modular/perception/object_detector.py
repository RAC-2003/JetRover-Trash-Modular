"""Object detection strategies behind one interface. Only responsibility:
given a frame, return a Detection. Never moves the robot, never touches
gripper/arm state - the old project's detection logic was inlined directly
in the state-machine callbacks, which is what made it untestable without a
live robot.

Three strategies, selected by detection.strategy in config.yaml:
  claude       - Claude does localization + material classification every call
  chatgpt      - same, via ChatGPT
  yolo_hybrid  - local YOLO does localization every call (no API, ~30ms);
                 Claude is asked for material only once per class, then cached
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class Detection:
    visible: bool
    center_x: Optional[int] = None
    center_y: Optional[int] = None
    bbox_w: Optional[int] = None
    bbox_h: Optional[int] = None
    material: Optional[str] = None
    confidence: float = 0.0

    @staticmethod
    def not_visible():
        return Detection(visible=False)


class VLMDetector:
    """Shared logic for the claude/chatgpt strategies - only the client class differs."""

    def __init__(self, client, classes, logger=None):
        self.client = client
        self.classes = classes
        self.logger = logger

    def detect(self, frame):
        height, width = frame.shape[:2]
        result = self.client.detect(frame, width, height, self.classes)
        if not result.get('object_visible'):
            return Detection.not_visible()
        return Detection(
            visible=True,
            center_x=result.get('center_x'),
            center_y=result.get('center_y'),
            bbox_w=result.get('bbox_w'),
            bbox_h=result.get('bbox_h'),
            material=result.get('material'),
            confidence=float(result.get('confidence', 0.0)),
        )


class YoloHybridDetector:
    def __init__(self, model_path, conf_threshold, classify_client, class_names=None, logger=None):
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        # Open-vocabulary YOLO checkpoints (YOLO-World, YOLOE) need their
        # class-name text embeddings set explicitly before inference - a
        # plain YOLO() load leaves the contrastive head with nothing to
        # compare against. A standard closed-set YOLO checkpoint has no
        # set_classes() at all, so this whole block is a no-op for those.
        #
        # YOLO-World's set_classes(names) (found via the generic YOLO()
        # loader) takes just the names and wires inference up correctly on
        # its own.
        #
        # YOLOE needs more, confirmed by reading the installed ultralytics
        # 8.3.143 source directly (nn/tasks.py, nn/modules/head.py):
        #   1. It needs the dedicated YOLOE wrapper class - set_classes()
        #      found via the generic loader is the raw YOLOEModel method.
        #   2. YOLOEModel.set_classes(names, embeddings) needs embeddings
        #      too (from get_text_pe(names)) - but it ONLY stores them
        #      (self.pe = embeddings); it does not touch the detection head.
        #   3. Ultralytics auto-fuses Conv+BN on the first predict() call,
        #      which switches the head to BNContrastiveHead.forward_fuse(x, w)
        #      - but w is only ever supplied if the head's OWN fuse(pe) has
        #      already baked the embeddings into plain conv weights,
        #      replacing BNContrastiveHead entirely. Nothing calls that
        #      automatically; YOLOEModel.get_vocab() calls it by hand
        #      (head.fuse(self.pe)) right after set_classes() for exactly
        #      this reason, so we do the same here.
        if class_names and hasattr(self.model, 'set_classes'):
            class_names = list(class_names)
            try:
                self.model.set_classes(class_names)
                if logger:
                    logger.info(f'[yolo_hybrid] YOLO-World model - set_classes({class_names})')
            except TypeError:
                from ultralytics import YOLOE
                self.model = YOLOE(model_path)
                embeddings = self.model.get_text_pe(class_names)
                self.model.set_classes(class_names, embeddings)
                yoloe_model = self.model.model  # underlying YOLOEModel
                device = next(yoloe_model.parameters()).device
                yoloe_model.model[-1].fuse(yoloe_model.pe.to(device))  # bake embeddings into the head
                if logger:
                    logger.info(f'[yolo_hybrid] YOLOE model - set_classes({class_names}), head fused')
        self.conf_threshold = conf_threshold
        self.classify_client = classify_client
        self.logger = logger
        self._material_cache = {}

    def detect(self, frame):
        results = self.model(frame, conf=self.conf_threshold, verbose=False)[0]
        if len(results.boxes) == 0:
            return Detection.not_visible()

        # Explicitly pick the highest-confidence box, not just boxes[0] -
        # when more than one candidate is in frame, box order isn't
        # guaranteed to be confidence-sorted or stable frame-to-frame, which
        # would otherwise let alignment silently track a different physical
        # object between calls.
        box = max(results.boxes, key=lambda b: float(b.conf[0]))
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        conf = float(box.conf[0])
        yolo_class = self.model.names[int(box.cls[0])]

        material = self._material_cache.get(yolo_class)
        if material is None:
            material = self.classify_client.classify_material(frame, yolo_class)
            if material is not None:
                self._material_cache[yolo_class] = material
                if self.logger:
                    self.logger.info(f'[yolo_hybrid] cached material for "{yolo_class}" = {material}')

        if material is None:
            if self.logger:
                self.logger.warn(f'[yolo_hybrid] could not classify "{yolo_class}" - treating as not visible')
            return Detection.not_visible()

        return Detection(
            visible=True, center_x=cx, center_y=cy,
            bbox_w=int(x2 - x1), bbox_h=int(y2 - y1),
            material=material, confidence=conf,
        )


def create_detector(config, logger=None):
    det_cfg = config.get('detection', {})
    vlm_cfg = config.get('vlm', {})
    strategy = det_cfg.get('strategy', 'claude')
    classes = det_cfg.get('classes', {})

    if strategy == 'claude':
        from trash_modular.intelligence.vlm import ClaudeVLMClient
        client = ClaudeVLMClient(
            vlm_cfg.get('claude_model', 'claude-opus-4-5'),
            vlm_cfg.get('max_tokens_detect', 80),
            vlm_cfg.get('max_tokens_classify', 40),
            logger=logger,
        )
        return VLMDetector(client, classes, logger=logger)

    if strategy == 'chatgpt':
        from trash_modular.intelligence.vlm import ChatGPTVLMClient
        client = ChatGPTVLMClient(
            vlm_cfg.get('chatgpt_model', 'gpt-4o'),
            vlm_cfg.get('max_tokens_detect', 80),
            vlm_cfg.get('max_tokens_classify', 40),
            logger=logger,
        )
        return VLMDetector(client, classes, logger=logger)

    if strategy == 'yolo_hybrid':
        from trash_modular.config.params import resolve_path
        from trash_modular.intelligence.vlm import ClaudeVLMClient
        yolo_cfg = det_cfg.get('yolo', {})
        classify_client = ClaudeVLMClient(
            vlm_cfg.get('claude_model', 'claude-opus-4-5'),
            vlm_cfg.get('max_tokens_detect', 80),
            vlm_cfg.get('max_tokens_classify', 40),
            logger=logger,
        )
        return YoloHybridDetector(
            resolve_path(config, yolo_cfg.get('model_path', 'models/best.pt')),
            yolo_cfg.get('conf_threshold', 0.60),
            classify_client,
            class_names=list(classes.keys()),
            logger=logger,
        )

    raise ValueError(f'Unknown detection.strategy "{strategy}" (expected claude|chatgpt|yolo_hybrid)')
