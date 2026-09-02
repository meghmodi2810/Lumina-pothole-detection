"""
Lumina Autonomous Path Planner & Avoidance Arbitrator
Implements PRD Section 4.3 & Section 5.3:
- Calculates lateral trajectory deviation if pothole is detected in vehicle path.
- Computes steering correction offset (d_theta = +/- 0.35) and throttle regulation.
- Detects lane-spanning blockages and triggers emergency stops.
- Renders dynamic augmented reality (AR) trajectory curves and HUD instruments.
"""

import cv2
import numpy as np

class PathPlanner:
    def __init__(self, lane_width_ratio=0.6, reaction_y_thresh=0.45):
        """
        Args:
            lane_width_ratio: Width of the driving corridor relative to frame width (default 60%).
            reaction_y_thresh: Normalized y-coordinate below which obstacles trigger avoidance (0=top, 1=bottom).
        """
        self.lane_width_ratio = lane_width_ratio
        self.reaction_y_thresh = reaction_y_thresh
        self.current_steering = 0.0  # -1.0 (full left) to +1.0 (full right)
        self.current_throttle = 45   # Base throttle (0-100)
        self.smoothed_steering = 0.0

    def plan(self, detections, base_steering=0.0, base_throttle=45):
        """
        Evaluates detected potholes and calculates the final steering and throttle vector.
        
        detections: list of dicts with 'bbox' [x1, y1, x2, y2], 'confidence', 'centroid_norm' [xc, yc], 'width_norm'
        
        Returns:
            telemetry: dict with steering, throttle, hazard_detected, evasion_mode, warning_level
        """
        hazard_detected = False
        evasion_offset = 0.0
        final_throttle = base_throttle
        evasion_mode = "CENTER_TRACKING"
        warning_level = "GREEN"  # GREEN, AMBER, RED
        critical_hazards = []

        # Driving corridor boundaries [x_left, x_right] centered in the lower frame
        corridor_x_min = (1.0 - self.lane_width_ratio) / 2.0
        corridor_x_max = 1.0 - corridor_x_min

        # Sort detections by proximity to vehicle (highest y_c first)
        sorted_dets = sorted(detections, key=lambda d: d.get('centroid_norm', [0.5, 0.0])[1], reverse=True)

        for det in sorted_dets:
            xc, yc = det.get('centroid_norm', [0.5, 0.0])
            w_norm = det.get('width_norm', 0.2)
            conf = det.get('confidence', 0.5)

            # Check if obstacle is close enough to matter (y_c > reaction threshold)
            if yc >= self.reaction_y_thresh and conf >= 0.35:
                # Check if obstacle is inside or encroaching on the vehicle path corridor
                box_x1 = det['bbox_norm'][0]
                box_x2 = det['bbox_norm'][2]
                
                overlaps_corridor = not (box_x2 < corridor_x_min or box_x1 > corridor_x_max)
                
                if overlaps_corridor:
                    hazard_detected = True
                    critical_hazards.append(det)

        if hazard_detected and critical_hazards:
            # Check for lane-spanning or multiple opposing obstacles
            primary = critical_hazards[0]
            w_norm = primary.get('width_norm', 0.0)
            xc, yc = primary.get('centroid_norm', [0.5, 0.0])

            # Condition 1: Lane-spanning hazard (w > 0.55 or multiple holes blocking both sides)
            left_blocked = any(d['centroid_norm'][0] < 0.50 for d in critical_hazards)
            right_blocked = any(d['centroid_norm'][0] >= 0.50 for d in critical_hazards)
            
            if w_norm >= 0.55 or (left_blocked and right_blocked and len(critical_hazards) >= 2):
                evasion_mode = "EMERGENCY_BRAKE"
                evasion_offset = 0.0
                final_throttle = 0
                warning_level = "RED"
            elif xc < 0.50:
                # Pothole on left side -> steer right (+0.35 offset) and decelerate by 30%
                evasion_mode = "EVADE_RIGHT"
                evasion_offset = +0.35
                final_throttle = int(base_throttle * 0.70)
                warning_level = "AMBER"
            else:
                # Pothole on right side -> steer left (-0.35 offset) and decelerate by 30%
                evasion_mode = "EVADE_LEFT"
                evasion_offset = -0.35
                final_throttle = int(base_throttle * 0.70)
                warning_level = "AMBER"

        target_steering = max(-1.0, min(1.0, base_steering + evasion_offset))
        # Exponential smoothing for realistic steering dynamics (alpha = 0.35)
        self.smoothed_steering = 0.65 * self.smoothed_steering + 0.35 * target_steering
        self.current_steering = round(float(self.smoothed_steering), 3)
        self.current_throttle = int(final_throttle)

        return {
            "steering": self.current_steering,
            "target_steering": round(float(target_steering), 3),
            "throttle": self.current_throttle,
            "hazard_detected": hazard_detected,
            "hazard_count": len(critical_hazards),
            "evasion_mode": evasion_mode,
            "warning_level": warning_level,
            "evasion_offset": evasion_offset
        }

    def render_hud(self, img, telemetry, fps=None, model_name=None):
        """
        Renders an augmented reality HUD on top of the camera frame:
        - Dynamic trajectory green/amber/red corridor curving with steering.
        - Steering angle compass bar.
        - Throttle speedometer bar.
        - Warning level banner.
        """
        h, w = img.shape[:2]
        hud_img = img.copy()

        # 1. Render Dynamic Augmented Reality Trajectory Ribbon
        self._draw_trajectory_corridor(hud_img, telemetry, w, h)

        # 2. Render Top HUD Bar (Dark semi-transparent header)
        top_bar_h = 52
        overlay = hud_img.copy()
        cv2.rectangle(overlay, (0, 0), (w, top_bar_h), (15, 18, 24), -1)
        cv2.addWeighted(overlay, 0.75, hud_img, 0.25, 0, hud_img)
        cv2.line(hud_img, (0, top_bar_h), (w, top_bar_h), (0, 240, 255), 1)

        # Telemetry status badge
        mode = telemetry.get("evasion_mode", "CENTER_TRACKING")
        warn = telemetry.get("warning_level", "GREEN")
        
        if warn == "RED":
            badge_color = (0, 0, 235)    # Red
            badge_text = "EMERGENCY HALT"
        elif warn == "AMBER":
            badge_color = (0, 180, 255)  # Amber
            badge_text = f"EVADING: {mode}"
        else:
            badge_color = (0, 230, 115)  # Neon Green
            badge_text = "AUTONOMOUS PATH CLEAR"

        # Draw status pill
        cv2.rectangle(hud_img, (14, 10), (220, 42), badge_color, -1)
        cv2.putText(hud_img, badge_text, (20, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2, cv2.LINE_AA)

        # Steering value & mini needle
        steer_val = telemetry.get("steering", 0.0)
        steer_pct = int(steer_val * 100)
        steer_str = f"STEER: {steer_val:+.2f} ({steer_pct:+}%)"
        cv2.putText(hud_img, steer_str, (235, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 240, 255), 1, cv2.LINE_AA)

        # Throttle value
        throttle_val = telemetry.get("throttle", 45)
        cv2.putText(hud_img, f"THROTTLE: {throttle_val}%", (440, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)

        # FPS & Model if provided
        right_offset = max(600, w - 240)
        if fps:
            cv2.putText(hud_img, f"FPS: {fps:.1f}", (right_offset, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 255, 100), 1, cv2.LINE_AA)
        if model_name:
            cv2.putText(hud_img, f"[{model_name}]", (right_offset + 95, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        return hud_img

    def _draw_trajectory_corridor(self, img, telemetry, w, h):
        """
        Draws the prospective vehicle driving corridor projected onto the road surface.
        Curves laterally in proportion to self.current_steering.
        """
        steer = self.current_steering
        warn = telemetry.get("warning_level", "GREEN")

        if warn == "RED":
            corridor_color = (0, 0, 240)    # Red
            alpha = 0.35
        elif warn == "AMBER":
            corridor_color = (0, 165, 255)  # Orange
            alpha = 0.30
        else:
            corridor_color = (0, 230, 115)  # Green
            alpha = 0.18

        # Trajectory coordinates in perspective
        y_bottom = h
        y_top = int(h * 0.52)

        x_center_bottom = w // 2
        lane_half_bottom = int(w * 0.28)
        
        # Lateral curve displacement at horizon
        curve_offset = int(steer * (w * 0.32))
        x_center_top = (w // 2) + curve_offset
        lane_half_top = int(w * 0.08)

        # Polygon points for drivable corridor ribbon
        pts = np.array([
            [x_center_bottom - lane_half_bottom, y_bottom],
            [x_center_top - lane_half_top, y_top],
            [x_center_top + lane_half_top, y_top],
            [x_center_bottom + lane_half_bottom, y_bottom]
        ], dtype=np.int32)

        # Draw translucent polygon fill
        overlay = img.copy()
        cv2.fillPoly(overlay, [pts], corridor_color)
        cv2.addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img)

        # Draw outer guideline borders
        cv2.line(img, (x_center_bottom - lane_half_bottom, y_bottom), (x_center_top - lane_half_top, y_top), corridor_color, 2, cv2.LINE_AA)
        cv2.line(img, (x_center_bottom + lane_half_bottom, y_bottom), (x_center_top + lane_half_top, y_top), corridor_color, 2, cv2.LINE_AA)

        # Draw dynamic center guidance dashes
        center_pts = [
            (x_center_bottom, y_bottom),
            (int(x_center_bottom * 0.65 + x_center_top * 0.35), int(y_bottom * 0.65 + y_top * 0.35)),
            (int(x_center_bottom * 0.35 + x_center_top * 0.65), int(y_bottom * 0.35 + y_top * 0.65)),
            (x_center_top, y_top)
        ]
        cv2.line(img, center_pts[0], center_pts[1], (255, 255, 255), 2, cv2.LINE_AA)
        cv2.line(img, center_pts[2], center_pts[3], (255, 255, 255), 2, cv2.LINE_AA)
