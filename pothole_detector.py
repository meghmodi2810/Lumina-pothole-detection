"""
Lumina Backward Compatibility Wrapper
Redirects legacy PotholeDetector calls to the high-accuracy PotholeEngine.
"""

import os
from pothole_engine import PotholeEngine

class PotholeDetector:
    def __init__(self, model_name="road_damage_seg_best.pt", models_dir="models"):
        self.engine = PotholeEngine(models_dir=models_dir, default_model=model_name)
        self.model = self.engine.model

    def detect_and_analyze(self, image_input, conf_threshold=0.25, enable_box_fusion=True):
        return self.engine.detect_and_analyze(
            image_input,
            conf_threshold=conf_threshold,
            enable_box_fusion=enable_box_fusion,
            is_video_stream=False
        )
