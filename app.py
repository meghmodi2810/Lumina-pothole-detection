"""
Lumina: Autonomous Self-Driving Smart Car Dashboard & Perception Microservice
Full real-time video stream, webcam inference, driving video analysis, and autonomous path planning.
"""

import os
import time
import json
import threading
import cv2
import numpy as np
from flask import Flask, render_template, request, jsonify, Response, send_from_directory

from pothole_engine import PotholeEngine
from path_planner import PathPlanner

app = Flask(__name__)

# Base directories
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
UPLOAD_FOLDER = os.path.join(BASE_DIR, "static", "uploads")
OUTPUT_FOLDER = os.path.join(BASE_DIR, "static", "outputs")
SAMPLE_IMG_FOLDER = os.path.join(BASE_DIR, "sample_images")
SAMPLE_VID_FOLDER = os.path.join(BASE_DIR, "sample_videos")
MODELS_FOLDER = os.path.join(BASE_DIR, "models")

for folder in [UPLOAD_FOLDER, OUTPUT_FOLDER, SAMPLE_IMG_FOLDER, SAMPLE_VID_FOLDER, MODELS_FOLDER]:
    os.makedirs(folder, exist_ok=True)

# Initialize Perception Engine & Autonomous Path Planner
engine = PotholeEngine(models_dir=MODELS_FOLDER, default_model="road_damage_seg_best.pt")
planner = PathPlanner()

# Global Live Stream State
class VideoCamera:
    def __init__(self):
        self.cap = None
        self.is_running = False
        self.mode = "webcam"  # "webcam" or "simulation"
        self.sim_video_path = os.path.join(SAMPLE_VID_FOLDER, "driving_sample.mp4")
        self.lock = threading.RLock()
        self.latest_telemetry = {
            "steering": 0.0,
            "throttle": 45,
            "warning_level": "GREEN",
            "evasion_mode": "CENTER_TRACKING",
            "hazard_count": 0,
            "fps": 0.0,
            "active_model": engine.active_model_name
        }

    def start(self, mode="webcam"):
        with self.lock:
            self.stop()
            self.mode = mode
            if mode == "webcam":
                # Try physical camera 0
                self.cap = cv2.VideoCapture(0)
                if not self.cap.isOpened():
                    print("[Camera] Physical webcam unavailable, falling back to simulated driving stream.")
                    self.mode = "simulation"
                    self.cap = cv2.VideoCapture(self.sim_video_path)
            else:
                self.cap = cv2.VideoCapture(self.sim_video_path)
            self.is_running = True

    def stop(self):
        with self.lock:
            self.is_running = False
            if self.cap is not None:
                self.cap.release()
                self.cap = None

    def get_frame(self, conf_thresh=0.30):
        with self.lock:
            if not self.is_running or self.cap is None or not self.cap.isOpened():
                return None, self.latest_telemetry

            ret, frame = self.cap.read()
            if not ret:
                if self.mode == "simulation":
                    # Loop video back to beginning for continuous simulation
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                if not ret:
                    return None, self.latest_telemetry

            return frame, self.latest_telemetry

camera = VideoCamera()

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/sample/<filename>")
def get_sample_image(filename):
    return send_from_directory(SAMPLE_IMG_FOLDER, filename)

@app.route("/sample_video/<filename>")
def get_sample_video(filename):
    return send_from_directory(SAMPLE_VID_FOLDER, filename)

@app.route("/api/samples", methods=["GET"])
def get_samples():
    """Lists available sample images and videos."""
    images = []
    if os.path.exists(SAMPLE_IMG_FOLDER):
        for f in sorted(os.listdir(SAMPLE_IMG_FOLDER)):
            if f.lower().endswith(('.jpg', '.jpeg', '.png')):
                images.append({
                    "name": f,
                    "url": f"/sample/{f}",
                    "title": f.replace("_", " ").replace(".jpg", "").replace(".png", "").title()
                })

    videos = []
    if os.path.exists(SAMPLE_VID_FOLDER):
        for f in sorted(os.listdir(SAMPLE_VID_FOLDER)):
            if f.lower().endswith(('.mp4', '.avi', '.mov')):
                videos.append({
                    "name": f,
                    "url": f"/sample_video/{f}",
                    "title": f.replace("_", " ").replace(".mp4", "").replace(".avi", "").title()
                })

    return jsonify({"images": images, "videos": videos})

@app.route("/api/models", methods=["GET"])
def list_models():
    """Returns all available model checkpoints and active model status."""
    return jsonify({
        "models": engine.get_available_models(),
        "active_model": engine.active_model_name,
        "is_segmentation": engine.is_segmentation
    })

@app.route("/api/models/switch", methods=["POST"])
def switch_model():
    """Switches the active model in real time."""
    data = request.get_json() or {}
    model_name = data.get("model_name", "")
    if not model_name:
        return jsonify({"error": "No model name provided"}), 400

    try:
        engine.load_model(model_name)
        camera.latest_telemetry["active_model"] = engine.active_model_name
        return jsonify({
            "success": True,
            "active_model": engine.active_model_name,
            "is_segmentation": engine.is_segmentation
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/api/telemetry", methods=["GET"])
def get_telemetry():
    """Returns real-time vehicle telemetry vector."""
    return jsonify(camera.latest_telemetry)

@app.route("/api/camera/toggle", methods=["POST"])
def toggle_camera():
    """Starts or stops the live camera / simulation stream."""
    data = request.get_json() or {}
    action = data.get("action", "start")
    mode = data.get("mode", "simulation")  # "webcam" or "simulation"

    if action == "start":
        camera.start(mode=mode)
        return jsonify({"status": "running", "mode": camera.mode})
    else:
        camera.stop()
        return jsonify({"status": "stopped"})

def generate_video_stream():
    """Generator for live multipart MJPEG stream with AI overlay and HUD."""
    prev_time = time.time()
    while True:
        if not camera.is_running:
            time.sleep(0.1)
            continue

        frame, _ = camera.get_frame()
        if frame is None:
            time.sleep(0.04)
            continue

        # Downsample large frames for silky smooth real-time streaming
        h, w = frame.shape[:2]
        if w > 800:
            scale = 640.0 / w
            frame_resized = cv2.resize(frame, (640, int(h * scale)))
        else:
            frame_resized = frame

        # Run AI Perception with temporal tracking
        t0 = time.time()
        annotated_img, detections, summary = engine.detect_and_analyze(
            frame_resized,
            conf_threshold=0.30,
            enable_box_fusion=True,
            is_video_stream=True
        )

        # Run Autonomous Avoidance Path Planner
        plan = planner.plan(detections)

        # Measure FPS
        fps = 1.0 / max(0.001, time.time() - prev_time)
        prev_time = time.time()

        # Render Dynamic AR Trajectory Ribbon + Instrument HUD
        hud_img = planner.render_hud(annotated_img, plan, fps=fps, model_name=engine.active_model_name)

        # Update latest telemetry cache
        camera.latest_telemetry = {
            "steering": plan["steering"],
            "target_steering": plan["target_steering"],
            "throttle": plan["throttle"],
            "warning_level": plan["warning_level"],
            "evasion_mode": plan["evasion_mode"],
            "hazard_count": len(detections),
            "fps": round(fps, 1),
            "active_model": engine.active_model_name,
            "inference_time_ms": summary["inference_time_ms"]
        }

        # Encode frame as JPEG
        ret, jpeg = cv2.imencode('.jpg', hud_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.route("/video_feed")
def video_feed():
    """MJPEG stream endpoint for live camera feed."""
    return Response(generate_video_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/api/detect", methods=["POST"])
def detect_image_api():
    """Static image detection API with autonomous avoidance path planning."""
    try:
        conf_thresh = float(request.form.get("confidence", 0.25))
        sample_name = request.form.get("sample_name", "")
        model_name = request.form.get("model_name", "")
        enable_box_fusion = request.form.get("enable_box_fusion", "true").lower() == "true"
        include_hud = request.form.get("include_hud", "true").lower() == "true"

        if model_name and model_name != engine.active_model_name:
            engine.load_model(model_name)

        img = None
        input_filename = ""

        if "image" in request.files and request.files["image"].filename != "":
            file = request.files["image"]
            timestamp = int(time.time() * 1000)
            ext = os.path.splitext(file.filename)[1] or ".jpg"
            input_filename = f"upload_{timestamp}{ext}"
            input_path = os.path.join(UPLOAD_FOLDER, input_filename)
            file.save(input_path)
            img = cv2.imread(input_path)
            original_url = f"/static/uploads/{input_filename}"
        elif sample_name:
            sample_path = os.path.join(SAMPLE_IMG_FOLDER, sample_name)
            if not os.path.exists(sample_path):
                return jsonify({"error": f"Sample '{sample_name}' not found"}), 404
            img = cv2.imread(sample_path)
            original_url = f"/sample/{sample_name}"
        else:
            return jsonify({"error": "No image uploaded or sample selected"}), 400

        if img is None:
            return jsonify({"error": "Failed to read image"}), 400

        # Perception inference
        annotated_img, detections, summary = engine.detect_and_analyze(
            img,
            conf_threshold=conf_thresh,
            enable_box_fusion=enable_box_fusion,
            is_video_stream=False
        )

        # Path planning & evasion calculation
        plan = planner.plan(detections)

        # Overlay HUD if requested
        if include_hud:
            final_img = planner.render_hud(annotated_img, plan, model_name=engine.active_model_name)
        else:
            final_img = annotated_img

        # Save output image
        timestamp = int(time.time() * 1000)
        output_filename = f"detected_{timestamp}.jpg"
        output_path = os.path.join(OUTPUT_FOLDER, output_filename)
        cv2.imwrite(output_path, final_img)
        detected_url = f"/static/outputs/{output_filename}"

        return jsonify({
            "success": True,
            "original_image_url": original_url,
            "detected_image_url": detected_url,
            "detections": detections,
            "summary": summary,
            "plan": plan
        })

    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

@app.route("/api/detect_video", methods=["POST"])
def detect_video_api():
    """Processes an uploaded video or sample video frame-by-frame with tracking."""
    try:
        conf_thresh = float(request.form.get("confidence", 0.30))
        sample_video_name = request.form.get("sample_video", "")
        model_name = request.form.get("model_name", "")

        if model_name and model_name != engine.active_model_name:
            engine.load_model(model_name)

        input_video_path = None
        orig_video_url = ""

        if "video" in request.files and request.files["video"].filename != "":
            file = request.files["video"]
            timestamp = int(time.time() * 1000)
            ext = os.path.splitext(file.filename)[1] or ".mp4"
            filename = f"upload_vid_{timestamp}{ext}"
            input_video_path = os.path.join(UPLOAD_FOLDER, filename)
            file.save(input_video_path)
            orig_video_url = f"/static/uploads/{filename}"
        elif sample_video_name:
            sample_path = os.path.join(SAMPLE_VID_FOLDER, sample_video_name)
            if not os.path.exists(sample_path):
                return jsonify({"error": f"Sample video '{sample_video_name}' not found"}), 404
            input_video_path = sample_path
            orig_video_url = f"/sample_video/{sample_video_name}"
        else:
            # Default to driving_sample.mp4
            input_video_path = os.path.join(SAMPLE_VID_FOLDER, "driving_sample.mp4")
            orig_video_url = "/sample_video/driving_sample.mp4"

        cap = cv2.VideoCapture(input_video_path)
        if not cap.isOpened():
            return jsonify({"error": "Failed to open video file"}), 400

        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        timestamp = int(time.time() * 1000)
        output_filename = f"processed_video_{timestamp}.mp4"
        output_path = os.path.join(OUTPUT_FOLDER, output_filename)

        frames_processed = 0
        frames_with_potholes = 0
        total_detections = 0
        evasion_events = 0
        emergency_brakes = 0

        start_time = time.time()

        # Process up to 60 frames for snappy web response on CPU
        max_frames_to_process = min(60, total_frames)
        
        # Scale output video to 640 width if original is larger
        if w > 800:
            out_w = 640
            out_h = int(h * (640.0 / w))
        else:
            out_w, out_h = w, h

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, fps, (out_w, out_h))

        while cap.isOpened() and frames_processed < max_frames_to_process:
            ret, frame = cap.read()
            if not ret:
                break

            if out_w != w:
                frame_eval = cv2.resize(frame, (out_w, out_h))
            else:
                frame_eval = frame

            ann_img, dets, summary = engine.detect_and_analyze(
                frame_eval,
                conf_threshold=conf_thresh,
                enable_box_fusion=True,
                is_video_stream=True
            )
            plan = planner.plan(dets)
            hud_frame = planner.render_hud(ann_img, plan, fps=fps, model_name=engine.active_model_name)
            out.write(hud_frame)

            if len(dets) > 0:
                frames_with_potholes += 1
                total_detections += len(dets)

            if plan["evasion_mode"] in ["EVADE_LEFT", "EVADE_RIGHT"]:
                evasion_events += 1
            elif plan["evasion_mode"] == "EMERGENCY_BRAKE":
                emergency_brakes += 1

            frames_processed += 1

        cap.release()
        out.release()

        elapsed_sec = round(time.time() - start_time, 2)
        avg_proc_fps = round(frames_processed / max(0.01, elapsed_sec), 1)

        return jsonify({
            "success": True,
            "original_video_url": orig_video_url,
            "processed_video_url": f"/static/outputs/{output_filename}",
            "metrics": {
                "frames_processed": frames_processed,
                "total_video_frames": total_frames,
                "frames_with_potholes": frames_with_potholes,
                "pothole_detection_rate_pct": round((frames_with_potholes / max(1, frames_processed)) * 100, 1),
                "total_detections_logged": total_detections,
                "evasion_events_count": evasion_events,
                "emergency_brake_events": emergency_brakes,
                "processing_time_sec": elapsed_sec,
                "processing_fps": avg_proc_fps,
                "active_model": engine.active_model_name
            }
        })

    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    print("\n=======================================================")
    print(" 🚀 Lumina Autonomous Pothole Detection & Vision Cockpit")
    print(f" 📦 Active Model : {engine.active_model_name} ({'Segmentation' if engine.is_segmentation else 'Detection'})")
    print(" 🌐 Server URL    : http://127.0.0.1:5000")
    print("=======================================================\n")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
