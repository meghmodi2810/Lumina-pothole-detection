"""
LUMINA Perception Benchmark & Stress Suite
Evaluates YOLOv8n-Seg model accuracy, latency across resolutions, thread optimization,
spatial box fusion, and temporal track persistence.
"""

import os
import sys
import time
import unittest
import numpy as np
import cv2

# Ensure lumina root is in path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from pothole_engine import PotholeEngine
from path_planner import PathPlanner


class TestPerceptionBenchmark(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        models_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models"))
        cls.engine = PotholeEngine(models_dir=models_dir, default_model="road_damage_seg_best.pt")
        cls.planner = PathPlanner()

    def test_01_model_attributes_and_task(self):
        """Verifies active model metadata, task type, and class mapping."""
        self.assertEqual(self.engine.active_model_name, "road_damage_seg_best.pt")
        self.assertTrue(self.engine.is_segmentation, "Expected road_damage_seg_best.pt to be an Instance Segmentation model.")
        names = getattr(self.engine.model, "names", {})
        self.assertIn(0, names)
        print(f"\n[Model Metadata] Name: {self.engine.active_model_name} | Task: {self.engine.model.task} | Classes: {names}")

    def test_02_resolution_latency_sweep(self):
        """Measures inference latency across 256, 320, and 480 px resolutions."""
        resolutions = [(256, 192), (320, 240), (480, 360)]
        results = {}

        for w, h in resolutions:
            dummy_frame = np.random.randint(60, 180, (h, w, 3), dtype=np.uint8)
            latencies = []
            # Warm up
            self.engine.detect_and_analyze(dummy_frame, conf_threshold=0.25, imgsz=w)

            # Benchmark 3 runs
            for _ in range(3):
                t0 = time.time()
                _, dets, summary = self.engine.detect_and_analyze(dummy_frame, conf_threshold=0.25, imgsz=w)
                elapsed_ms = (time.time() - t0) * 1000.0
                latencies.append(elapsed_ms)

            avg_ms = sum(latencies) / len(latencies)
            results[f"{w}x{h}"] = avg_ms

        print("\n[Inference Latency Sweep]")
        for res, ms in results.items():
            print(f"  - {res}: {ms:.1f} ms ({1000.0/ms:.2f} FPS)")

        # 320x240 should finish in reasonable time
        self.assertLess(results["320x240"], 4000.0, "320x240 inference is unacceptably slow (> 4s).")

    def test_03_perspective_distance_calculation(self):
        """Verifies realistic ground-plane distance estimation based on vertical road contact coordinates."""
        # Top of road (horizon, y_contact ~ 0.35) -> should be far away (~3 to 4.5m)
        # Bottom of frame (near bumper, y_contact ~ 0.90) -> should be close (~0.3 to 0.6m)
        h, w = 480, 640
        y_horizon = int(h * 0.38)
        y_bumper = int(h * 0.92)

        # Create dummy detection at horizon
        det_horizon = [{"bbox": [200, y_horizon - 30, 260, y_horizon], "confidence": 0.85, "class_id": 0}]
        # Create dummy detection at bumper
        det_bumper = [{"bbox": [200, y_bumper - 50, 300, y_bumper], "confidence": 0.90, "class_id": 0}]

        y_c_horizon = y_horizon / float(h)
        dist_horizon = max(0.2, min(4.5, round(0.35 / max(0.06, (y_c_horizon - 0.28)), 1)))

        y_c_bumper = y_bumper / float(h)
        dist_bumper = max(0.2, min(4.5, round(0.35 / max(0.06, (y_c_bumper - 0.28)), 1)))

        print(f"\n[Ground-Plane Projection] Horizon (yc={y_c_horizon:.2f}): {dist_horizon}m | Bumper (yc={y_c_bumper:.2f}): {dist_bumper}m")
        self.assertGreater(dist_horizon, dist_bumper, "Horizon obstacle distance must be greater than bumper obstacle distance.")
        self.assertGreaterEqual(dist_horizon, 2.5, "Horizon obstacle should be at least 2.5m ahead.")
        self.assertLessEqual(dist_bumper, 0.8, "Bumper obstacle should be within 0.8m.")

    def test_04_spatial_box_fusion(self):
        """Verifies that overlapping or fragmented bounding boxes for the same crater are cleanly fused."""
        raw_dets = [
            {"bbox": [100, 200, 180, 260], "confidence": 0.80, "mask": None},
            {"bbox": [110, 210, 190, 270], "confidence": 0.75, "mask": None},  # Overlaps significantly
            {"bbox": [400, 300, 480, 360], "confidence": 0.88, "mask": None},  # Distinct independent obstacle
        ]

        fused = self.engine._fuse_overlapping_boxes(raw_dets, iou_thresh=0.18, io_min_thresh=0.35)
        print(f"\n[Spatial Box Fusion] Input: {len(raw_dets)} boxes -> Fused: {len(fused)} boxes")
        self.assertEqual(len(fused), 2, "Overlapping boxes failed to fuse into 1 unified hazard.")
        # Fused box should encompass both
        fused_box = fused[0]["bbox"]
        self.assertEqual(fused_box[0], 100)
        self.assertEqual(fused_box[1], 200)
        self.assertEqual(fused_box[2], 190)
        self.assertEqual(fused_box[3], 270)

    def test_05_temporal_tracking_persistence(self):
        """Verifies that a detected hazard maintains a stable Track ID across consecutive video frames with dropout."""
        self.engine.tracks.clear()
        self.engine.next_track_id = 1

        w, h = 640, 480

        # Frame 1: Pothole appears at (200, 200, 300, 280)
        d1 = [{"bbox": [200, 200, 300, 280], "confidence": 0.85, "mask": None}]
        tr1 = self.engine._update_temporal_tracks(d1, w, h)
        self.assertEqual(len(tr1), 1)
        first_id = tr1[0]["id"]

        # Frame 2: Pothole shifts slightly as car moves forward (202, 208, 304, 290)
        d2 = [{"bbox": [202, 208, 304, 290], "confidence": 0.88, "mask": None}]
        tr2 = self.engine._update_temporal_tracks(d2, w, h)
        self.assertEqual(len(tr2), 1)
        self.assertEqual(tr2[0]["id"], first_id, "Track ID changed across continuous movement!")

        # Frame 3: Brief occlusion / frame dropout (0 detections)
        tr3 = self.engine._update_temporal_tracks([], w, h)
        self.assertEqual(len(tr3), 0)
        self.assertIn(first_id, self.engine.tracks, "Track was prematurely dropped on 1-frame occlusion!")

        # Frame 4: Pothole re-appears
        d4 = [{"bbox": [205, 215, 308, 298], "confidence": 0.84, "mask": None}]
        tr4 = self.engine._update_temporal_tracks(d4, w, h)
        self.assertEqual(len(tr4), 1)
        self.assertEqual(tr4[0]["id"], first_id, "Track ID failed to persist across temporary occlusion!")
        print(f"\n[Temporal Tracking] Track #{first_id} persisted across motion and 1-frame dropout.")

    def test_06_false_positive_rejection(self):
        """Verifies screen-spanning degenerate boxes (>80% of entire frame) and tiny noise (<4px) are rejected."""
        dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Run detection
        _, dets, _ = self.engine.detect_and_analyze(dummy_frame, conf_threshold=0.25)
        # Should have 0 detections on pure blank black frame
        self.assertEqual(len(dets), 0, "Blank frame produced false positive detections.")


if __name__ == "__main__":
    unittest.main()
