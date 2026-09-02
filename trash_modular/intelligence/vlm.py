"""VLM API clients. Only responsibility: talk to Claude/ChatGPT and parse the
JSON response. No ROS, no robot state, no movement - callers (object_detector,
perception.bin_detector) decide what to do with the result, which makes this
mockable in tests without spinning a node or making a real API call.
"""

import base64
import json

import cv2


def build_detect_prompt(width, height, classes):
    """classes: {label: material} - e.g. {'apple': 'non-recyclable'}."""
    class_lines = '\n'.join(f'- "{label}" -> material = "{material}"' for label, material in classes.items())
    return f'''Look at this robot camera image ({width}px wide, {height}px tall).
Is one of the following objects visible that the robot could pick up?
{class_lines}

The object must be clearly visible, standalone, and graspable by a robot arm.

If a qualifying object is visible, respond with JSON only:
{{"object_visible": true, "center_x": <pixel x 0-{width}>, "center_y": <pixel y 0-{height}>, "bbox_w": <width px>, "bbox_h": <height px>, "material": "<material from the list above>", "confidence": <0.0-1.0>}}

If no qualifying object visible:
{{"object_visible": false}}

JSON only, no other text.'''


def build_classify_material_prompt(label):
    return (
        f'A robot vision system detected an object classified as "{label}". '
        'For this recycling-sorting task, is it recyclable or non-recyclable? '
        'Respond with JSON only: {"material": "recyclable" or "non-recyclable"}'
    )


def build_bin_prompt(width, material):
    return f'''Look at this robot camera image ({width}px wide).
The robot is holding a {material} item and needs to find the correct bin.
Is there a bin/container visible appropriate for {material} waste?
If the correct bin is visible, respond with JSON only:
{{"bin_visible": true, "bin_center_x": <pixel x 0-{width}>, "bin_width_px": <width px>}}
If not visible:
{{"bin_visible": false}}

JSON only, no other text.'''


def _parse_json_response(text):
    text = text.strip()
    if '```' in text:
        text = text.split('```')[1]
        if text.startswith('json'):
            text = text[4:]
    return json.loads(text.strip())


def _validate_pixel(result, width, height):
    if not result.get('object_visible'):
        return result
    px, py = result.get('center_x'), result.get('center_y')
    if px is None or py is None or not (0 <= px <= width) or not (0 <= py <= height):
        return {'object_visible': False}
    return result


def _validate_bin_pixel(result, width):
    if not result.get('bin_visible'):
        return result
    px = result.get('bin_center_x')
    if px is None or not (0 <= px <= width):
        return {'bin_visible': False}
    return result


def _encode_jpeg_b64(frame):
    _, buffer = cv2.imencode('.jpg', frame)
    return base64.standard_b64encode(buffer).decode('utf-8')


class _VLMClientBase:
    """Shared detect/classify_material/detect_bin logic. Subclasses only need
    to implement _call() - the actual provider API request/response shape."""

    def __init__(self, max_tokens_detect=80, max_tokens_classify=40, logger=None):
        self.max_tokens_detect = max_tokens_detect
        self.max_tokens_classify = max_tokens_classify
        self.logger = logger

    def _call(self, frame, prompt, max_tokens):
        raise NotImplementedError

    def detect(self, frame, width, height, classes):
        try:
            prompt = build_detect_prompt(width, height, classes)
            text = self._call(frame, prompt, self.max_tokens_detect)
            result = _parse_json_response(text)
            return _validate_pixel(result, width, height)
        except Exception as e:
            if self.logger:
                self.logger.error(f'{type(self).__name__}.detect error: {e}')
            return {'object_visible': False}

    def classify_material(self, frame, label):
        try:
            prompt = build_classify_material_prompt(label)
            text = self._call(frame, prompt, self.max_tokens_classify)
            result = _parse_json_response(text)
            material = result.get('material')
            return material if material in ('recyclable', 'non-recyclable') else None
        except Exception as e:
            if self.logger:
                self.logger.error(f'{type(self).__name__}.classify_material error: {e}')
            return None

    def detect_bin(self, frame, material, width):
        """Used by perception.bin_detector when bins.location_mode is
        'detect' - same client/model/credentials as trash detection, just a
        different prompt asking for a bin instead of a trash item."""
        try:
            prompt = build_bin_prompt(width, material)
            text = self._call(frame, prompt, self.max_tokens_detect)
            result = _parse_json_response(text)
            return _validate_bin_pixel(result, width)
        except Exception as e:
            if self.logger:
                self.logger.error(f'{type(self).__name__}.detect_bin error: {e}')
            return {'bin_visible': False}


class ClaudeVLMClient(_VLMClientBase):
    def __init__(self, model, max_tokens_detect=80, max_tokens_classify=40, logger=None):
        super().__init__(max_tokens_detect, max_tokens_classify, logger)
        import anthropic
        self.client = anthropic.Anthropic()
        self.model = model

    def _call(self, frame, prompt, max_tokens):
        image_b64 = _encode_jpeg_b64(frame)
        response = self.client.messages.create(
            model=self.model,
            max_tokens=max_tokens,
            messages=[{
                'role': 'user',
                'content': [
                    {'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg', 'data': image_b64}},
                    {'type': 'text', 'text': prompt},
                ],
            }],
        )
        return response.content[0].text


class ChatGPTVLMClient(_VLMClientBase):
    def __init__(self, model, max_tokens_detect=80, max_tokens_classify=40, logger=None):
        super().__init__(max_tokens_detect, max_tokens_classify, logger)
        import openai
        self.client = openai.OpenAI()
        self.model = model

    def _call(self, frame, prompt, max_tokens):
        image_b64 = _encode_jpeg_b64(frame)
        response = self.client.chat.completions.create(
            model=self.model,
            max_tokens=max_tokens,
            messages=[{
                'role': 'user',
                'content': [
                    {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{image_b64}'}},
                    {'type': 'text', 'text': prompt},
                ],
            }],
        )
        return response.choices[0].message.content
