"""
Lumina Advanced Pothole Perception Engine
Features:
- Dual Pipeline: Real-Time Bounding Box Detection + Instance Segmentation.
- Spatial Box Fusion (IoU & Containment Clustering) to eliminate partial/fragmented boxes.
- Temporal Object Tracking across video/webcam frames with persistent Track IDs.
- Dynamic Road Hazard Assessment (Centroid, Width, Severity, Approach Velocity).
- Multi-Model Dynamic Switching (Road Damage Seg, PeterHdd YOLOv8s, Keremberke, Legacy).
"""

import os
import time
import math
import cv2
import numpy as np
import torch
from ultralytics import YOLO

class PotholeTrack:
    """Represents a tracked pothole across consecutive video frames."""
    def __init__(self, track_id, bbox, conf, mask=None):
        self.track_id = track_id
        self.bbox = [float(c) for c in bbox]  # [x1, y1, x2, y2]
        self.conf = conf
        self.mask = mask
        self.misses = 0
        self.hits = 1
        self.last_seen = time.time()
        self.history = [self.get_centroid()]

    def get_centroid(self):
        return ((self.bbox[0] + self.bbox[2]) / 2.0, (self.bbox[1] + self.bbox[3]) / 2.0)

    def update(self, new_bbox, new_conf, new_mask=None, alpha=0.60):
        # Exponential moving average for smooth bounding box rendering
        for i in range(4):
            self.bbox[i] = alpha * float(new_bbox[i]) + (1.0 - alpha) * self.bbox[i]
        self.conf = 0.5 * self.conf + 0.5 * new_conf
        if new_mask is not None:
            self.mask = new_mask
        self.hits += 1
        self.misses = 0
        self.last_seen = time.time()
        self.history.append(self.get_centroid())
        if len(self.history) > 15:
            self.history.pop(0)


class PotholeEngine:
    def __init__(self, models_dir="models", default_model="road_damage_seg_best.pt"):
        self.models_dir = models_dir
        os.makedirs(self.models_dir, exist_ok=True)
        
        self.active_model_name = default_model
        self.model = None
        self.is_segmentation = False
        
        # Temporal Tracking State
        self.tracks = {}
        self.next_track_id = 1
        self.max_misses = 6  # Persist across frame dropouts for up to 6 frames

        # Load initial model
        self.load_model(default_model)

    def get_available_models(self):
        """Returns a list of all detected model weight files in /models."""
        available = []
        if os.path.exists(self.models_dir):
            for f in os.listdir(self.models_dir):
                if f.endswith(".pt") or f.endswith(".onnx"):
                    path = os.path.join(self.models_dir, f)
                    size_mb = round(os.path.getsize(path) / (1024 * 1024), 1)
                    is_seg = "seg" in f.lower()
                    desc = "Instance Segmentation (High-Accuracy Contours)" if is_seg else "Bounding Box Detection"
                    available.append({
                        "filename": f,
                        "size_mb": size_mb,
                        "type": "segmentation" if is_seg else "detection",
                        "description": desc,
                        "is_active": (f == self.active_model_name)
                    })
        return available

    def load_model(self, model_name):
        """Loads or switches to a specific model."""
        model_path = os.path.join(self.models_dir, model_name)
        if not os.path.exists(model_path):
            # Fallbacks
            fallbacks = ["road_damage_seg_best.pt", "peterhdd_best.pt", "pothole_seg_nano.pt", "pothole_best.pt", "yolov8n.pt"]
            for fb in fallbacks:
                alt = os.path.join(self.models_dir, fb)
                if os.path.exists(alt):
                    model_path = alt
                    model_name = fb
                    break

        print(f"[PotholeEngine] Loading model: {model_path}...")
        self.model = YOLO(model_path)
        self.active_model_name = model_name
        
        # Check task type
        task = getattr(self.model, "task", "")
        self.is_segmentation = ("segment" in str(task).lower()) or ("seg" in model_name.lower())
        print(f"[PotholeEngine] Ready! Model: {model_name} | Task: {'Segmentation' if self.is_segmentation else 'Detection'}")
        # Reset tracks on model switch
        self.tracks.clear()
        self.next_track_id = 1
        return True

    def detect_and_analyze(self, image_input, conf_threshold=0.25, enable_box_fusion=True, is_video_stream=False):
        """
        Runs comprehensive perception on a single frame.
        
        Returns:
            annotated_img (numpy array): Image with HUD, bounding boxes, or segmentation masks
            detections (list): Structured list of detected potholes
            summary (dict): Telemetry and road condition statistics
        """
        if isinstance(image_input, str):
            img = cv2.imread(image_input)
            if img is None:
                raise ValueError(f"Could not read image: {image_input}")
        elif isinstance(image_input, np.ndarray):
            img = image_input.copy()
        else:
            raise ValueError("Input must be filepath or numpy ndarray")

        h, w = img.shape[:2]
        start_time = time.time()

        # Run YOLO inference
        results = self.model.predict(img, conf=conf_threshold, verbose=False)[0]
        inference_time_ms = round((time.time() - start_time) * 1000, 1)

        raw_detections = []
        has_masks = results.masks is not None and len(results.masks) > 0

        # 1. Parse Raw Model Outputs
        if len(results.boxes) > 0:
            for idx, box in enumerate(results.boxes):
                coords = [int(round(c)) for c in box.xyxy[0].tolist()]
                conf = float(box.conf[0].item())
                cls_id = int(box.cls[0].item())
                
                # Boundary clamping
                x1, y1 = max(0, coords[0]), max(0, coords[1])
                x2, y2 = min(w, coords[2]), min(h, coords[3])
                if x2 - x1 <= 4 or y2 - y1 <= 4:
                    continue

                mask_contour = None
                if has_masks and idx < len(results.masks):
                    try:
                        # Extract polygon contour points
                        poly = results.masks.xy[idx]
                        if len(poly) > 2:
                            mask_contour = np.array(poly, dtype=np.int32)
                    except Exception:
                        mask_contour = None

                raw_detections.append({
                    "bbox": [x1, y1, x2, y2],
                    "confidence": conf,
                    "mask": mask_contour,
                    "class_id": cls_id
                })

        # 2. Apply Spatial Box Fusion if enabled (merges fragmented boxes)
        if enable_box_fusion and len(raw_detections) > 1 and not has_masks:
            fused_detections = self._fuse_overlapping_boxes(raw_detections, iou_thresh=0.18, io_min_thresh=0.35)
        else:
            fused_detections = raw_detections

        # 3. Apply Temporal Tracking (if video stream) or Direct Assignment
        if is_video_stream:
            tracked_detections = self._update_temporal_tracks(fused_detections, w, h)
        else:
            tracked_detections = []
            for i, d in enumerate(fused_detections, 1):
                d["id"] = i
                tracked_detections.append(d)

        # 4. Compute Physical Metrics (Centroid, Severity, Lane Intrusion)
        detections = []
        annotated_img = img.copy()
        mask_overlay = annotated_img.copy()

        for det in tracked_detections:
            x1, y1, x2, y2 = det["bbox"]
            conf = det["confidence"]
            det_id = det.get("id", 1)
            mask = det.get("mask", None)

            bw = x2 - x1
            bh = y2 - y1
            area_pixels = bw * bh
            area_pct = round((area_pixels / (w * h)) * 100, 2)
            
            # Normalized centroid & dimensions
            xc = round(((x1 + x2) / 2.0) / w, 3)
            yc = round(((y1 + y2) / 2.0) / h, 3)
            wn = round(bw / w, 3)
            hn = round(bh / h, 3)

            # Severity classification
            if area_pct >= 2.5 or (yc >= 0.65 and conf >= 0.70) or wn >= 0.35:
                severity = "Critical"
                box_color = (0, 0, 240)     # Neon Red
                fill_color = (0, 0, 190)
            elif area_pct >= 0.8 or (yc >= 0.45 and conf >= 0.50):
                severity = "Moderate"
                box_color = (0, 160, 255)   # Amber Orange
                fill_color = (0, 120, 220)
            else:
                severity = "Minor"
                box_color = (0, 225, 255)   # Vivid Yellow
                fill_color = (0, 180, 210)

            # Render Instance Segmentation Mask if available
            if mask is not None and len(mask) > 2:
                cv2.fillPoly(mask_overlay, [mask], fill_color)
                cv2.polylines(annotated_img, [mask], isClosed=True, color=box_color, thickness=2, lineType=cv2.LINE_AA)
            else:
                # Render semi-transparent box fill
                cv2.rectangle(mask_overlay, (x1, y1), (x2, y2), fill_color, -1)

            # Draw custom cyber-styled bounding box with corner highlights
            label = f"Pothole #{det_id} | {conf*100:.0f}% ({severity})"
            self._draw_cyber_box(annotated_img, (x1, y1, x2, y2), box_color, label)

            detections.append({
                "id": det_id,
                "class": "Pothole",
                "confidence": round(conf, 4),
                "confidence_percent": round(conf * 100, 1),
                "bbox": [x1, y1, x2, y2],
                "bbox_norm": [round(x1/w, 3), round(y1/h, 3), round(x2/w, 3), round(y2/h, 3)],
                "centroid_norm": [xc, yc],
                "width_norm": wn,
                "height_norm": hn,
                "area_pct": area_pct,
                "severity": severity,
                "has_mask": mask is not None
            })

        # Blend semi-transparent highlight layer (22% alpha)
        cv2.addWeighted(mask_overlay, 0.22, annotated_img, 0.78, 0, annotated_img)

        # Sort detections by proximity (highest y_c first)
        detections.sort(key=lambda d: d["centroid_norm"][1], reverse=True)

        # Summary statistics
        total = len(detections)
        avg_conf = round(float(np.mean([d["confidence"] for d in detections])) * 100, 1) if total > 0 else 0.0
        crit = sum(1 for d in detections if d["severity"] == "Critical")
        mod = sum(1 for d in detections if d["severity"] == "Moderate")
        min_c = sum(1 for d in detections if d["severity"] == "Minor")

        if total == 0:
            road_status = "Good Condition (Road Clear)"
            risk = "Low (0/10)"
        elif crit > 0 or total >= 3:
            road_status = "Hazardous - Immediate Evasion Needed"
            risk = "High (8.5/10)"
        else:
            road_status = "Fair - Maintenance Recommended"
            risk = "Moderate (5.0/10)"

        summary = {
            "total_potholes": total,
            "avg_confidence_pct": avg_conf,
            "critical_count": crit,
            "moderate_count": mod,
            "minor_count": min_c,
            "road_condition": road_status,
            "risk_index": risk,
            "inference_time_ms": inference_time_ms,
            "resolution": f"{w}x{h}",
            "active_model": self.active_model_name,
            "model_type": "Instance Segmentation" if self.is_segmentation else "Object Detection"
        }

        return annotated_img, detections, summary

    def _fuse_overlapping_boxes(self, raw_dets, iou_thresh=0.18, io_min_thresh=0.35):
        """Fuses adjacent or overlapping boxes of the same hazard into a single bounding box."""
        boxes = [d["bbox"] for d in raw_dets]
        confs = [d["confidence"] for d in raw_dets]
        masks = [d.get("mask", None) for d in raw_dets]
        merged = [False] * len(boxes)
        clusters = []

        for i in range(len(boxes)):
            if merged[i]:
                continue
            cluster = [i]
            merged[i] = True
            for j in range(i + 1, len(boxes)):
                if merged[j]:
                    continue
                iou = self._compute_iou(boxes[i], boxes[j])
                iomin = self._compute_io_min(boxes[i], boxes[j])
                # Merge if IoU is above threshold or one box largely contains the other
                if iou > iou_thresh or iomin > io_min_thresh:
                    cluster.append(j)
                    merged[j] = True
            clusters.append(cluster)

        fused = []
        for cl in clusters:
            f_x1 = min(boxes[idx][0] for idx in cl)
            f_y1 = min(boxes[idx][1] for idx in cl)
            f_x2 = max(boxes[idx][2] for idx in cl)
            f_y2 = max(boxes[idx][3] for idx in cl)
            # Slight boost for multi-anchor consensus
            f_conf = min(0.99, max(confs[idx] for idx in cl) * (1.0 + 0.04 * (len(cl) - 1)))
            
            # Use mask of highest confidence if present
            best_idx = max(cl, key=lambda idx: confs[idx])
            best_mask = masks[best_idx]

            fused.append({
                "bbox": [f_x1, f_y1, f_x2, f_y2],
                "confidence": round(f_conf, 4),
                "mask": best_mask,
                "fused_count": len(cl)
            })
        return fused

    def _update_temporal_tracks(self, detections, w, h):
        """Matches detections against existing tracks to ensure stable IDs across video frames."""
        matched_track_ids = set()
        updated_detections = []

        # Age existing tracks
        for tid, track in list(self.tracks.items()):
            track.misses += 1

        for det in detections:
            bbox = det["bbox"]
            conf = det["confidence"]
            mask = det.get("mask")
            c_det = ((bbox[0] + bbox[2]) / 2.0, (bbox[1] + bbox[3]) / 2.0)

            best_tid = None
            best_iou = 0.0
            best_dist = 9999.0

            for tid, track in self.tracks.items():
                if tid in matched_track_ids:
                    continue
                iou = self._compute_iou(bbox, track.bbox)
                c_tr = track.get_centroid()
                dist = math.hypot(c_det[0] - c_tr[0], c_det[1] - c_tr[1])
                
                # Normalize distance by frame diagonal
                norm_dist = dist / math.hypot(w, h)

                if iou > 0.20 or norm_dist < 0.08:
                    if iou > best_iou or norm_dist < best_dist:
                        best_iou = iou
                        best_dist = norm_dist
                        best_tid = tid

            if best_tid is not None:
                self.tracks[best_tid].update(bbox, conf, mask)
                matched_track_ids.add(best_tid)
                det["id"] = best_tid
                det["bbox"] = [int(round(c)) for c in self.tracks[best_tid].bbox]
            else:
                # Spawn new track
                new_id = self.next_track_id
                self.next_track_id += 1
                self.tracks[new_id] = PotholeTrack(new_id, bbox, conf, mask)
                matched_track_ids.add(new_id)
                det["id"] = new_id

            updated_detections.append(det)

        # Prune dead tracks
        for tid in list(self.tracks.keys()):
            if self.tracks[tid].misses > self.max_misses:
                del self.tracks[tid]

        return updated_detections

    def _draw_cyber_box(self, img, bbox, color, label):
        """Draws clean HUD styling with sharp corner accents and label badge."""
        x1, y1, x2, y2 = bbox
        thickness = 2

        # Main bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

        # Corner brackets
        corner_len = min(18, int((x2 - x1) * 0.22), int((y2 - y1) * 0.22))
        cv2.line(img, (x1, y1), (x1 + corner_len, y1), color, thickness + 2)
        cv2.line(img, (x1, y1), (x1, y1 + corner_len), color, thickness + 2)
        cv2.line(img, (x2, y1), (x2 - corner_len, y1), color, thickness + 2)
        cv2.line(img, (x2, y1), (x2, y1 + corner_len), color, thickness + 2)
        cv2.line(img, (x1, y2), (x1 + corner_len, y2), color, thickness + 2)
        cv2.line(img, (x1, y2), (x1, y1 - corner_len if y1 > y2 else y2 - corner_len), color, thickness + 2)
        cv2.line(img, (x2, y2), (x2 - corner_len, y2), color, thickness + 2)
        cv2.line(img, (x2, y2), (x2, y2 - corner_len), color, thickness + 2)

        # Label Pill
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.48
        font_thickness = 1
        (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
        
        label_y1 = max(0, y1 - text_h - 8)
        cv2.rectangle(img, (x1, label_y1), (x1 + text_w + 10, label_y1 + text_h + 8), color, -1)
        cv2.putText(img, label, (x1 + 5, label_y1 + text_h + 3), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

    @staticmethod
    def _compute_iou(box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        a1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        a2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = a1 + a2 - inter
        return inter / union if union > 0 else 0.0

    @staticmethod
    def _compute_io_min(box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        a1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        a2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        min_a = min(a1, a2)
        return inter / min_a if min_a > 0 else 0.0
