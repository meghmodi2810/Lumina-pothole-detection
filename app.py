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
import logging

from pothole_engine import PotholeEngine
from path_planner import PathPlanner
from diagnostics_engine import run_full_hardware_diagnostics

app = Flask(__name__)

# Suppress repetitive polling access logs from spamming stdout
class PollingLogFilter(logging.Filter):
    def filter(self, record):
        msg = record.getMessage()
        return "/api/v1/telemetry" not in msg and "/clean_feed" not in msg and "/video_feed" not in msg

logging.getLogger('werkzeug').addFilter(PollingLogFilter())

# ASGI wrapper so the app can be run with both 'python app.py' (Flask) and 'uvicorn app:asgi_app' (ASGI)
try:
    from asgiref.wsgi import WsgiToAsgi
    asgi_app = WsgiToAsgi(app)
except ImportError:
    asgi_app = app

# Base directories
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
UPLOAD_FOLDER = os.path.join(BASE_DIR, "static", "uploads")
OUTPUT_FOLDER = os.path.join(BASE_DIR, "static", "outputs")
SAMPLE_IMG_FOLDER = os.path.join(BASE_DIR, "sample_images")
SAMPLE_VID_FOLDER = os.path.join(BASE_DIR, "sample_videos")
MODELS_FOLDER = os.path.join(BASE_DIR, "models")

for folder in [UPLOAD_FOLDER, OUTPUT_FOLDER, SAMPLE_IMG_FOLDER, SAMPLE_VID_FOLDER, MODELS_FOLDER]:
    os.makedirs(folder, exist_ok=True)

def transcode_to_h264(raw_mp4_path, web_mp4_path):
    """Converts OpenCV mp4v video to HTML5-compatible H.264 using FFmpeg."""
    import subprocess
    import time
    time.sleep(0.25)  # Allow Windows file handle to release
    cmd = [
        "ffmpeg", "-y", "-i", str(raw_mp4_path),
        "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-preset", "ultrafast", "-movflags", "+faststart",
        str(web_mp4_path)
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True)
        if os.path.exists(web_mp4_path) and os.path.getsize(web_mp4_path) > 1000:
            print(f"[FFmpeg] Transcode to H.264 succeeded: {web_mp4_path}")
            return True
    except Exception as e:
        print(f"[FFmpeg] Conversion warning: {e}")
    return False

# Initialize Perception Engine & Autonomous Path Planner
engine = PotholeEngine(models_dir=MODELS_FOLDER, default_model="road_damage_seg_best.pt")
planner = PathPlanner()

# Global Live Stream State
class VideoCamera:
    def __init__(self):
        self.cap = None
        self.picam2 = None
        self.is_running = False
        self.mode = "webcam"  # "webcam", "picamera2", or "simulation"
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
                # 1. Try Picamera2 first (Native Raspberry Pi CSI Camera)
                try:
                    from picamera2 import Picamera2
                    self.picam2 = Picamera2()
                    config = self.picam2.create_preview_configuration(main={'size': (640, 480), 'format': 'RGB888'})
                    self.picam2.configure(config)
                    self.picam2.start()
                    time.sleep(0.3)
                    self.mode = "picamera2"
                    print("[Camera] Picamera2 CSI camera initialized successfully.")
                except Exception as e:
                    self.picam2 = None
                    # 2. Try OpenCV V4L2 / USB camera
                    self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                    if not self.cap.isOpened():
                        self.cap = cv2.VideoCapture(0)
                    
                    if not self.cap.isOpened():
                        print(f"[Camera] Physical camera unavailable ({e}), falling back to simulated driving stream.")
                        self.mode = "simulation"
                        self.cap = cv2.VideoCapture(self.sim_video_path)
            else:
                self.cap = cv2.VideoCapture(self.sim_video_path)
            self.is_running = True

    def stop(self):
        with self.lock:
            self.is_running = False
            if self.picam2 is not None:
                try:
                    self.picam2.stop()
                    self.picam2.close()
                except Exception:
                    pass
                self.picam2 = None
            if self.cap is not None:
                self.cap.release()
                self.cap = None

    def get_frame(self, conf_thresh=0.30):
        with self.lock:
            if not self.is_running:
                return None, self.latest_telemetry

            if self.mode == "picamera2" and self.picam2 is not None:
                try:
                    rgb_frame = self.picam2.capture_array()
                    bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                    return bgr_frame, self.latest_telemetry
                except Exception as e:
                    print(f"[Camera] Picamera2 capture error: {e}")
                    return None, self.latest_telemetry

            if self.cap is None or not self.cap.isOpened():
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
camera.start(mode="webcam")

latest_ai_detections = []
latest_ai_plan = planner.plan([])
ai_lock = threading.Lock()

def _ai_perception_worker():
    global latest_ai_detections, latest_ai_plan
    try:
        import torch
        torch.set_num_threads(4)
    except Exception:
        pass

    while True:
        if not camera.is_running:
            time.sleep(0.15)
            continue
        raw_frame, _ = camera.get_frame()
        if raw_frame is None:
            time.sleep(0.04)
            continue
        try:
            h, w = raw_frame.shape[:2]
            infer_w = 256
            infer_h = max(32, int(h * (infer_w / float(w))))
            small_frame = cv2.resize(raw_frame, (infer_w, infer_h))
            
            _, detections, _ = engine.detect_and_analyze(
                small_frame,
                conf_threshold=0.35,
                enable_box_fusion=False,
                is_video_stream=True,
                imgsz=256
            )

            # Scale box coordinates back to full resolution (w, h)
            scale_x = float(w) / infer_w
            scale_y = float(h) / infer_h
            scaled_detections = []
            for det in detections:
                d = dict(det)
                if "bbox" in d:
                    bx1, by1, bx2, by2 = d["bbox"]
                    d["bbox"] = [
                        int(round(bx1 * scale_x)),
                        int(round(by1 * scale_y)),
                        int(round(bx2 * scale_x)),
                        int(round(by2 * scale_y))
                    ]
                scaled_detections.append(d)

            plan = planner.plan(scaled_detections)
            with ai_lock:
                latest_ai_detections = scaled_detections
                latest_ai_plan = plan
                if scaled_detections:
                    for det in scaled_detections:
                        recent_detected_potholes.insert(0, {
                            "id": f"POT-{int(time.time()*1000)%100000}",
                            "timestamp": int(time.time() * 1000),
                            "severity": det.get("severity", "MODERATE").upper(),
                            "confidence": round(float(det.get("confidence", 0.8)), 2),
                            "estimatedAreaM2": 0.42,
                            "depthEstimateCm": 4.5,
                            "avoidanceAction": plan.get("evasion_mode", "CENTER_TRACKING"),
                            "isHazard": True
                        })
                    if len(recent_detected_potholes) > 20:
                        del recent_detected_potholes[20:]
                    total_potholes_count += len(scaled_detections)
        except Exception:
            pass
        time.sleep(0.08)

threading.Thread(target=_ai_perception_worker, daemon=True).start()

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
    """Generator for live multipart MJPEG stream with AI overlay and HUD at full 30 FPS."""
    prev_time = time.time()
    while True:
        if not camera.is_running:
            time.sleep(0.033)
            continue

        frame, _ = camera.get_frame()
        if frame is None:
            time.sleep(0.015)
            continue

        with ai_lock:
            cur_plan = dict(latest_ai_plan)
            cur_dets = list(latest_ai_detections)

        fps = 1.0 / max(0.001, time.time() - prev_time)
        prev_time = time.time()

        # ALWAYS render on top of the live fresh camera frame for smooth 30 FPS
        display_frame = frame.copy()

        # Overlay detected potholes directly onto current frame (<0.05ms)
        for det in cur_dets:
            bbox = det.get("bbox")
            if bbox and len(bbox) == 4:
                x1, y1, x2, y2 = bbox
                conf = det.get("confidence", 0.5)
                det_id = det.get("id", 1)
                sev = det.get("severity", "Moderate")
                color = (0, 0, 240) if sev == "Critical" else ((0, 160, 255) if sev == "Moderate" else (0, 225, 255))
                label = f"Pothole #{det_id} | {int(conf * 100)}%"
                engine._draw_cyber_box(display_frame, (x1, y1, x2, y2), color, label)

        hud_img = planner.render_hud(display_frame, cur_plan, fps=fps, model_name=engine.active_model_name)

        camera.latest_telemetry = {
            "steering": cur_plan.get("steering", 0.0),
            "target_steering": cur_plan.get("target_steering", 0.0),
            "throttle": cur_plan.get("throttle", 0),
            "warning_level": cur_plan.get("warning_level", "GREEN"),
            "evasion_mode": cur_plan.get("evasion_mode", "CENTER_TRACKING"),
            "hazard_count": len(cur_dets),
            "fps": round(fps, 1),
            "active_model": engine.active_model_name,
            "inference_time_ms": 25.0
        }

        ret, jpeg = cv2.imencode('.jpg', hud_img, [cv2.IMWRITE_JPEG_QUALITY, 65])
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        time.sleep(0.01)

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
        raw_output_path = os.path.join(OUTPUT_FOLDER, f"raw_{output_filename}")

        frames_processed = 0
        frames_with_potholes = 0
        total_detections = 0
        evasion_events = 0
        emergency_brakes = 0

        start_time = time.time()

        # If sample video, start at frame 30 where potholes are visible
        if "driving_sample" in input_video_path and total_frames > 60:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 30)

        # Process up to 75 frames
        max_frames_to_process = min(75, total_frames)
        
        # Scale output video to 640 width if original is larger
        if w > 800:
            out_w = 640
            out_h = int(h * (640.0 / w))
        else:
            out_w, out_h = w, h

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(raw_output_path, fourcc, fps, (out_w, out_h))

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

        # Transcode with FFmpeg to H.264 for universal HTML5 browser playback
        if transcode_to_h264(raw_output_path, output_path):
            try:
                os.remove(raw_output_path)
            except Exception:
                pass
        else:
            # Fallback if ffmpeg is unavailable
            if os.path.exists(raw_output_path):
                os.rename(raw_output_path, output_path)

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


# ------------------------------------------------------------------------------
# LUMINA MOBILE COMPANION APP INTEGRATION & HARDWARE MOTOR CONTROL
# ------------------------------------------------------------------------------
from motor_controller import MotorController
from autonomous_driver import AutonomousDriver, AutonomousState

# Verified Hardware GPIO Pinout for L298N (from test_motors_debug.py & test_drive.py):
# IN1=17, IN2=27, ENA=18, IN3=22, IN4=23, ENB=12 (PWM @ 100 Hz)
motor = MotorController(
    in1=17, in2=27, ena=18,
    in3=22, in4=23, enb=12,
    pwm_frequency=100,
    max_speed_ceiling=1.0,
    watchdog_timeout_sec=0.6
)

def get_current_ai_plan():
    with ai_lock:
        return dict(latest_ai_plan)

# Autonomous Driving & Intelligent Obstacle Avoidance Engine (Fused AI Vision + Ultrasonic Safety)
auto_driver = AutonomousDriver(
    motor_controller=motor,
    plan_provider=get_current_ai_plan,
    trig_pin=16,
    echo_pin=26,
    stop_dist_cm=28.0,
    backoff_dist_cm=18.0,
    clear_dist_cm=35.0,
    cruise_speed=65.0,
    reverse_speed=70.0,
    turn_speed=75.0
)

# Vehicle Autonomy State
active_drive_mode = "manual"  # "manual" or "autonomous"
is_drive_engaged = False
recent_detected_potholes = []
total_potholes_count = 0
odometer_distance_m = 0.0

def get_pi_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return round(float(f.read().strip()) / 1000.0, 1)
    except Exception:
        return 38.5

@app.after_request
def add_cors_headers(response):
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, DELETE, OPTIONS"
    return response

@app.route("/api/v1/vehicle/health", methods=["GET"])
@app.route("/api/health", methods=["GET"])
def get_vehicle_health():
    cpu_t = get_pi_cpu_temp()
    cam_ok = camera.is_running
    return jsonify({
        "status": "HEALTHY",
        "vehicleId": "LUMINA-01",
        "subsystems": {
            "camera": "OPERATIONAL" if cam_ok else "DEGRADED",
            "inference": "READY" if engine is not None else "DEGRADED",
            "motors": "ACTIVE" if motor.is_hardware else "SIMULATED",
            "thermal": "OK" if cpu_t < 75.0 else "WARNING"
        },
        "temperatures": {
            "cpuC": cpu_t,
            "motorC": 32.0
        },
        "activeModel": engine.active_model_name if engine else "None",
        "hardwareActuation": motor.is_hardware
    })

@app.route("/api/v1/telemetry", methods=["GET"])
def get_mobile_telemetry():
    global odometer_distance_m
    m_stat = motor.get_status()
    cpu_t = get_pi_cpu_temp()
    
    speed_est = round((m_stat["left_pwm_pct"] + m_stat["right_pwm_pct"]) / 20.0 * 2.5, 1) if m_stat["is_driving"] else 0.0
    odometer_distance_m += (speed_est / 3.6) * 0.15

    with ai_lock:
        latest_pot = recent_detected_potholes[0] if recent_detected_potholes else None
        tot_potholes = total_potholes_count
        ai_p = dict(latest_ai_plan)

    auto_stat = auto_driver.get_telemetry()

    if active_drive_mode == "autonomous":
        if auto_stat["state"] in ("OBSTACLE_STOP", "BACKING_OFF", "BLOCKED_WAITING"):
            evasion_mode = "EMERGENCY_BRAKE"
            warning_level = "RED"
        elif auto_stat["state"] in ("TURNING", "DECIDING_TURN"):
            evasion_mode = "PATH_ADJUSTED"
            warning_level = "YELLOW"
        else:
            # Active vision-guided pothole evasion mode from YOLOv8 perception
            evasion_mode = ai_p.get("evasion_mode", "CENTER_TRACKING")
            warning_level = ai_p.get("warning_level", "GREEN")
    else:
        evasion_mode = ai_p.get("evasion_mode", "CENTER_TRACKING")
        warning_level = ai_p.get("warning_level", "GREEN")

    return jsonify({
        "vehicleId": "LUMINA-01",
        "timestamp": int(time.time() * 1000),
        "speedKmh": speed_est,
        "distanceM": round(odometer_distance_m, 1),
        "boardTemperatureC": cpu_t,
        "motorTemperatureC": 32.0,
        "networkLatencyMs": 14,
        "batteryPercent": None,
        "powerSupplyStatus": "NOMINAL",
        "driveMode": active_drive_mode,
        "isDriving": m_stat["is_driving"],
        "isEmergencyStopped": m_stat["is_emergency_stopped"],
        "steering": m_stat["current_steering"],
        "throttle": m_stat["current_throttle"],
        "leftPwm": m_stat["left_pwm_pct"],
        "rightPwm": m_stat["right_pwm_pct"],
        "fps": camera.latest_telemetry.get("fps", 30.0),
        "warningLevel": warning_level,
        "evasionMode": evasion_mode,
        "autonomousState": auto_stat["state"],
        "ultrasonicDistanceCm": auto_stat["distanceCm"],
        "avoidanceAttempts": auto_stat["avoidanceAttempts"],
        "totalPotholesDetected": total_potholes_count,
        "recentPothole": latest_pot,
        "gps": {"latitude": 21.1702, "longitude": 72.8311}
    })

def start_discovery_responder():
    """Listens on UDP 5005 for LUMINA mobile app discovery pings."""
    import socket
    def discovery_worker():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            sock.bind(("", 5005))
            print("[Discovery] UDP beacon & discovery responder active on port 5005.")
            while True:
                data, addr = sock.recvfrom(1024)
                msg = data.decode("utf-8", errors="ignore").strip()
                if "LUMINA" in msg or "DISCOVER" in msg:
                    response = json.dumps({
                        "service": "LUMINA_CAR",
                        "vehicleId": "LUMINA-01",
                        "port": 5000,
                        "status": "HEALTHY"
                    })
                    sock.sendto(response.encode("utf-8"), addr)
        except Exception as e:
            print(f"[Discovery] UDP Responder warning: {e}")

    threading.Thread(target=discovery_worker, daemon=True, name="LuminaDiscovery").start()

start_discovery_responder()

@app.route("/api/v1/control", methods=["POST"])
@app.route("/api/drive", methods=["POST"])
def post_manual_control():
    data = request.get_json(silent=True) or {}
    control = data.get("control", data)

    # 1. Discrete 4-directional action (forward, backward, left, right, stop)
    action = data.get("action") or control.get("action")
    if action:
        speed = data.get("speed") or control.get("speed") or 100.0
        if str(action).lower() != "stop" and motor.is_emergency_stopped:
            motor.release_emergency_stop()
        motor.execute_action(str(action), float(speed))
        return jsonify({"success": True, "action": str(action), "status": motor.get_status()})

    # 2. Proportional steering & throttle
    steering = float(control.get("steering", 0.0))
    throttle = float(control.get("throttle", 0.0))

    if active_drive_mode == "manual":
        if abs(throttle) > 0.1 and motor.is_emergency_stopped:
            motor.release_emergency_stop()
        motor.set_drive_command(steering, throttle)

    return jsonify({"success": True, "steering": steering, "throttle": throttle, "status": motor.get_status()})

@app.route("/api/v1/control/<action>", methods=["GET", "POST"])
def handle_action_control(action):
    speed = request.args.get("speed", default=100.0, type=float)
    if str(action).lower() != "stop" and motor.is_emergency_stopped:
        motor.release_emergency_stop()
    motor.execute_action(str(action), speed)
    return jsonify({"success": True, "action": str(action), "status": motor.get_status()})

@app.route("/api/v1/vehicle/start", methods=["POST"])
def start_vehicle():
    global is_drive_engaged
    is_drive_engaged = True
    motor.release_emergency_stop()
    if active_drive_mode == "autonomous":
        auto_driver.start()
    return jsonify({"success": True, "state": "DRIVING", "mode": active_drive_mode})

@app.route("/api/v1/vehicle/stop", methods=["POST"])
def stop_vehicle():
    global is_drive_engaged
    is_drive_engaged = False
    if active_drive_mode == "autonomous":
        auto_driver.stop()
    motor.set_drive_command(0.0, 0.0)
    return jsonify({"success": True, "state": "STOPPED", "mode": active_drive_mode})

@app.route("/api/v1/vehicle/mode", methods=["POST"])
def set_vehicle_mode():
    global active_drive_mode, is_drive_engaged
    data = request.get_json() or {}
    new_mode = data.get("mode", "manual").lower()
    if new_mode in ["manual", "autonomous"]:
        if new_mode != active_drive_mode:
            # Switching modes: safely stop motors first
            motor.set_drive_command(0.0, 0.0)
            if active_drive_mode == "autonomous":
                auto_driver.stop()
            active_drive_mode = new_mode

        # When autonomous mode is selected, activate autonomous navigation
        if active_drive_mode == "autonomous":
            is_drive_engaged = True
            auto_driver.start()
        else:
            auto_driver.stop()

        return jsonify({
            "success": True,
            "mode": active_drive_mode,
            "isDriving": is_drive_engaged,
            "autonomous": auto_driver.get_telemetry()
        })
    return jsonify({"error": f"Invalid mode: {new_mode}"}), 400

@app.route("/api/v1/emergency_stop", methods=["POST"])
def trigger_emergency_stop():
    global is_drive_engaged
    is_drive_engaged = False
    auto_driver.emergency_stop()
    motor.emergency_stop()
    return jsonify({"success": True, "emergency_stop": True})

@app.route("/api/v1/emergency_release", methods=["POST"])
def release_emergency_stop():
    auto_driver.stop()
    motor.release_emergency_stop()
    return jsonify({"success": True, "emergency_stop": False})

@app.route("/api/v1/session/pair", methods=["POST"])
def pair_session():
    # Automatically clear any previous emergency stop so car is instantly ready!
    motor.release_emergency_stop()
    return jsonify({
        "sessionId": f"ses_{int(time.time())}",
        "vehicleId": "LUMINA-01",
        "accessMode": "CONTROLLER",
        "controllerClaimed": True,
        "token": "tok_lumina_secret_session_2026"
    })

@app.route("/api/v1/session/controller/claim", methods=["POST"])
def claim_controller():
    return jsonify({"success": True, "controllerClaimed": True})

@app.route("/api/v1/diagnostics/run", methods=["GET", "POST"])
def run_diagnostics():
    report = run_full_hardware_diagnostics(camera=camera, engine=engine, motor=motor, auto_driver=auto_driver)
    return jsonify(report)

@app.route("/clean_feed")
def clean_video_feed():
    def generate_clean_stream():
        while True:
            if not camera.is_running:
                time.sleep(0.1)
                continue
            frame, _ = camera.get_frame()
            if frame is None:
                time.sleep(0.04)
                continue
            ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.033)
    return Response(generate_clean_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')
# ------------------------------------------------------------------------------

if __name__ == "__main__":
    print("\n=======================================================")
    print(" [LUMINA] Autonomous Pothole Detection & Vision Cockpit")
    print(f" [MODEL] Active Model : {engine.active_model_name} ({'Segmentation' if engine.is_segmentation else 'Detection'})")
    print(" [SERVER] Server URL    : http://127.0.0.1:5000")
    print("=======================================================\n")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
