# Project Lumina: Autonomous Smart Car Perception & Pothole Avoidance System

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![Ultralytics YOLOv8](https://img.shields.io/badge/YOLOv8-Instance_Segmentation-cyan.svg)](https://docs.ultralytics.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

An end-to-end autonomous smart car perception and navigation platform designed for real-time road hazard detection, precise polygon pothole segmentation, temporal hazard tracking, and dynamic autonomous steering avoidance.

---

## 1. System Architecture

Project Lumina operates as a **Hybrid Edge-Cloud Cyber-Physical System**:

```
+----------------------------------------------------------------------------------------+
|                             EDGE VEHICLE (Raspberry Pi 4B)                             |
|                                                                                        |
|  +---------------------+        +--------------------+        +---------------------+  |
|  |    Pi Camera V2     |        |   HC-SR04 Sensor   |        |    NEO-6M GPS       |  |
|  | (1080p @ 30 FPS)    |        | (50Hz Safety Loop) |        | (NMEA Telemetry)    |  |
|  +----------+----------+        +---------+----------+        +----------+----------+  |
|             |                             |                              |             |
|             v                             v                              v             |
|  +----------------------------------------------------------------------------------+  |
|  |                          Lumina Edge Client (edge_client.py)                     |  |
|  | - Captures downsampled frames (640x360 or 320x240)                               |  |
|  | - Evaluates local 50Hz ultrasonic proximity safety cutoff (< 20 cm = HALT)       |  |
|  | - Encodes JPEG buffer & streams over 5GHz Wi-Fi / LTE to Perception Core        |  |
|  +----------------------------------------+-----------------------------------------+  |
+-------------------------------------------|--------------------------------------------+
                                            | (Sub-15ms WebSocket / HTTP POST)
                                            v
+----------------------------------------------------------------------------------------+
|                     PERCEPTION CORE & AVOIDANCE ENGINE (Local / AWS EC2)               |
|                                                                                        |
|  +----------------------------------------------------------------------------------+  |
|  | 1. High-Accuracy Instance Segmentation (YOLOv8n-Seg: road_damage_seg_best.pt)    |  |
|  |    Extracts exact polygon contours, aspect ratios, and bounding coordinates.    |  |
|  +----------------------------------------+-----------------------------------------+  |
|                                           |                                            |
|                                           v                                            |
|  +----------------------------------------------------------------------------------+  |
|  | 2. Spatial Box Fusion (pothole_engine.py)                                       |  |
|  |    Clusters fragmented asphalt micro-boxes via IoU & containment thresholds.     |  |
|  +----------------------------------------+-----------------------------------------+  |
|                                           |                                            |
|                                           v                                            |
|  +----------------------------------------------------------------------------------+  |
|  | 3. Temporal Object Tracking & Smoothing (EMA Filter)                             |  |
|  |    Assigns persistent Track IDs; filters frame jitter; tracks approach rate.     |  |
|  +----------------------------------------+-----------------------------------------+  |
|                                           |                                            |
|                                           v                                            |
|  +----------------------------------------------------------------------------------+  |
|  | 4. Autonomous Avoidance Arbitrator (path_planner.py)                             |  |
|  |    Computes lateral steering delta (-0.35 to +0.35), throttle %, and halts.      |  |
|  +----------------------------------------+-----------------------------------------+  |
|                                           |                                            |
|                                           v                                            |
|  +----------------------------------------------------------------------------------+  |
|  | 5. Real-Time Vision Cockpit & Telemetry API (app.py, templates/index.html)        |  |
|  |    Streams MJPEG / H.264 video with dynamic AR driving corridors and HUD meters. |  |
|  +----------------------------------------+-----------------------------------------+  |
+-------------------------------------------|--------------------------------------------+
                                            |
                                            | Telemetry Vector: {"steer": +0.35, "throttle": 28}
                                            v
+----------------------------------------------------------------------------------------+
|                              ACTUATION & MOTOR CONTROL                                 |
|                                                                                        |
|  +----------------------------------------------------------------------------------+  |
|  | Differential Motor PWM Driver (L298N / TB6612FNG)                                |  |
|  | PWM_left  = Throttle * (1.0 + 0.40 * Steer)                                      |  |
|  | PWM_right = Throttle * (1.0 - 0.40 * Steer)                                      |  |
|  +----------------------------------------------------------------------------------+  |
+----------------------------------------------------------------------------------------+
```

---

## 2. Hardware Bill of Materials (BOM)

| Component | Specification | Function |
| :--- | :--- | :--- |
| **Compute Board** | Raspberry Pi 4B (4GB or 8GB) | Edge perception client, motor PWM generation, sensor polling |
| **Camera** | Raspberry Pi Camera Module V2 (Sony IMX219) | 1080p video feed, mounted at 12–15 cm height, 30 deg downward tilt |
| **Motors** | 4x 6V-12V Metal Gear DC Motors (200 RPM) | 4WD differential chassis actuation |
| **Motor Driver** | L298N Dual H-Bridge (or TB6612FNG MOSFET) | Direction and PWM speed modulation |
| **Safety Sensor** | HC-SR04 Ultrasonic Sensor | 50Hz hardware override safety cutoff (< 20 cm = Emergency Halt) |
| **Localization** | NEO-6M GPS Module | Global coordinate tagging for pothole geo-inventory (outdoor) |
| **Power A** | 7.4V 2S LiPo Battery | Dedicated motor drive power supply |
| **Power B** | 5V 3A Step-Down Buck Converter (LM2596) | Isolated clean logic power for Raspberry Pi 4B |

---

## 3. Computer Vision & Avoidance Pipeline

### 3.1 Instance Segmentation Core (`road_damage_seg_best.pt`)
Rather than crude bounding boxes that clip diagonal road cracks, Lumina uses **YOLOv8n-Seg** to extract exact polygon boundaries.
* **Model Size**: 6.8 MB (ideal for edge CPU and embedded devices).
* **Inference Speed**: ~12–15 ms on CPU (sub-5 ms on GPU / TensorRT).
* **Classes**: Road defects, craters, and potholes.

### 3.2 Spatial Box Fusion
Solves the legacy "fragmented box" failure where a single pothole is broken into 5–7 slices:
1. Calculates Intersection over Union (IoU) and containment overlap between raw candidates.
2. Merges overlapping clusters into a single convex hull and unified bounding perimeter.

### 3.3 Temporal EMA Tracking
* Assigns persistent Track IDs across video frames.
* Coordinates are smoothed using an Exponential Moving Average:
  $$\text{pos}_t = \alpha \cdot \text{pos}_{\text{new}} + (1 - \alpha) \cdot \text{pos}_{t-1}$$
* Prevents flickering, false dropouts, and sudden steering jerks.

### 3.4 Avoidance Arbitrator Policy
Implemented in `path_planner.py` following Section 4.3 of the PRD:

| Condition | Centroid ($X_c, Y_c$) | Action | Steering ($\Delta\theta$) | Throttle | AR Corridor Color |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Clear Road** | No hazards in zone | `CENTER_TRACKING` | `0.00` | `45%` | Emerald Green |
| **Hazard on Left** | $X_c < 0.50$, $Y_c \ge 0.45$ | `EVADE_RIGHT` | `+0.35` (Veer right) | `31%` | Amber Warning |
| **Hazard on Right** | $X_c \ge 0.50$, $Y_c \ge 0.45$ | `EVADE_LEFT` | `-0.35` (Veer left) | `31%` | Amber Warning |
| **Lane Blockage** | Both sides blocked or $W > 0.55$ | `EMERGENCY_BRAKE` | `0.00` | `0%` | Crimson Red |

---

## 4. Repository Structure

```
Project Lumina/
│
├── app.py                       # Vision server, MJPEG streaming & FFmpeg H.264 engine
├── autonomous_driver.py         # Multi-state closed-loop autonomous driving engine
├── diagnostics_engine.py        # Active hardware startup self-test & status LED engine
├── path_planner.py              # Avoidance arbitrator & dynamic AR trajectory renderer
├── pothole_engine.py            # Perception core: segmentation, box fusion, temporal tracking
├── motor_controller.py          # 4WD L298N motor driver with hardware PWM & watchdog
├── test_model_guided_driving.py # Verification suite for autonomous and manual model guidance
├── test_autonomous_driver.py    # Unit tests for autonomous driving state machine
├── AUTONOMOUS_HARDWARE_INTEGRATION_README.md # Detailed edge integration & model profiling artifact
├── pothole_detector.py          # Backward-compatibility API wrapper
├── train.py                     # Custom fine-tuning script with vehicular augmentations
├── export_onnx.py               # ONNX exporter with graph simplification
├── pyproject.toml               # Project metadata and dependencies
├── .gitignore                   # Excludes heavy binaries and temporary outputs
│
├── models/
│   ├── README.md                # Model documentation and weights guide
│   └── road_damage_seg_best.pt  # High-accuracy YOLOv8n-seg model (6.8 MB)
│
├── sample_images/               # Standardized benchmark images
│   ├── sample_pothole_1.jpg     # Single close pothole
│   ├── sample_pothole_2.jpg     # Dual overlapping hazards
│   └── sample_smooth_road.jpg   # Clean road control (0 false positives)
│
├── sample_videos/
│   └── driving_sample.mp4       # Real dashcam driving footage for video analysis
│
├── templates/
│   └── index.html               # Sleek cyber-robotics vision cockpit dashboard
│
└── DOCUMENTS/
    └── LUMINA_PRD_COMPREHENSIVE.md  # Complete Product Requirements Document
```

---

## 5. Getting Started

### 5.1 Prerequisites
* Python 3.10 or higher
* FFmpeg installed in system PATH (for H.264 web video streaming)

### 5.2 Local Installation
```powershell
# 1. Clone the repository
git clone https://github.com/meghmodi2810/Lumina-pothole-detection.git
cd Lumina-pothole-detection

# 2. Create and activate a virtual environment
python -m venv .venv
.\.venv\Scripts\activate       # On Linux/macOS: source .venv/bin/activate

# 3. Install core dependencies
pip install ultralytics torch opencv-python flask asgiref uvicorn requests
```

### 5.3 Launch the Vision Cockpit
```powershell
python app.py
```
Open your browser and navigate to **`http://127.0.0.1:5000`**.

---

## 6. Cloud & Edge Deployment (AWS EC2)

### 6.1 Deploying the Cloud Server
1. Launch an AWS EC2 instance (`t3.medium` or `c6i.large`) with Ubuntu 24.04 LTS.
2. In Security Groups, open **Port 22 (SSH)** and **Port 5000 (Custom TCP)**.
3. SSH into the instance, clone this repo, and copy `models/road_damage_seg_best.pt` to the `models/` directory:
   ```powershell
   scp -i "key.pem" models/road_damage_seg_best.pt ubuntu@<EC2_IP>:/home/ubuntu/Lumina-pothole-detection/models/
   ```
4. Run the server with Uvicorn or Systemd:
   ```bash
   python3 app.py
   ```

### 6.2 Running the Raspberry Pi Edge Client
On your Raspberry Pi 4B, run the lightweight client to capture camera frames and stream them to the cloud:
```python
import cv2, requests, time

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

AWS_ENDPOINT = "http://<YOUR_EC2_IP>:5000/api/detect"

while True:
    ret, frame = cap.read()
    if not ret: continue
    
    _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    files = {'image': ('frame.jpg', buffer.tobytes(), 'image/jpeg')}
    data = {'confidence': 0.22, 'include_hud': False}
    
    res = requests.post(AWS_ENDPOINT, files=files, data=data, timeout=0.8)
    if res.status_code == 200:
        telemetry = res.json()
        print("Avoidance telemetry:", telemetry.get("summary"))
    time.sleep(0.05)
```

---

## 7. API Reference

### `GET /api/telemetry`
Returns instantaneous vehicle dynamics, active avoidance recommendations, and warning state.
```json
{
  "steering": 0.35,
  "throttle": 31,
  "evasion_mode": "EVADE_RIGHT",
  "warning_level": "AMBER",
  "hazard_count": 1,
  "latency_ms": 14.2
}
```

### `POST /api/detect`
Performs full instance segmentation and trajectory planning on a static frame.
* **Form-data**: `image` (binary file), `confidence` (float, default 0.22), `include_hud` (bool).
* **Response**: Returns detections, centroid normalized coordinates, severity classification, and base64/URL annotated visualization.

### `POST /api/detect_video`
Batches driving video footage, tracks defects across time, and transcodes output to web-compatible H.264 video.

### `GET /video_feed`
Real-time multipart MJPEG stream rendering live detection polygons, bounding boxes, and dynamic AR path trajectory overlay.

### `POST /api/v1/vehicle/resume`
Resumes autonomous driving from an obstacle halt, trapped state (`BLOCKED_WAITING`), or emergency release without deadlock.

### `POST /api/v1/camera/start`
Actively restarts or wakes up the edge camera pipeline (CSI Picamera2 or V4L2 USB).

---

## 8. Autonomous Navigation & Hardware Diagnostics

For full documentation on the physical edge car architecture, see [AUTONOMOUS_HARDWARE_INTEGRATION_README.md](AUTONOMOUS_HARDWARE_INTEGRATION_README.md).

### 8.1 Hardware Startup Diagnostics (`/api/v1/diagnostics/run`)
- Checks MicroSD storage health (`shutil.disk_usage`), HC-SR04 sonar responsiveness (TRIG=16, ECHO=26), CSI camera capture (`picamera2`), YOLOv8 model residency in RAM, and SoC thermal ceiling (< 80°C).
- On all checks passing, pulses the **Green Status LED on GPIO 5 for 3.0 seconds**.
- Surfaces actionable subsystem failure diagnostics if any sensor or bus is degraded.

### 8.2 Closed-Loop Autonomous Driving (`/api/v1/vehicle/mode`)
- **Situation A (Clear Path)**: Cruises forward at calibrated 78% PWM duty cycle with **180ms Kickstart Breakout Pulse** to overcome static floor friction.
- **Situation B (Obstacle Detected)**: Immediate motor halt when distance $\le 20.0\text{ cm}$.
- **Situation C (Intelligent Evasion)**: Backs up 0.70s if close ($< 15.0\text{ cm}$), adaptively picks turn direction with a 0.70s high-torque pivot turn (88% PWM), avoiding repetitive loops, and verifies clearance $\ge 26.0\text{ cm}$.
- **Situation D (Trapped / Blocked)**: Halts in `BLOCKED_WAITING` after 5 failed turns, resuming automatically when path clears or immediately upon tapping Resume in the mobile app.
- **Brownout Protection & Stiction Buster**: Modulates hardware PWM on `ENA` (GPIO 18) and `ENB` (GPIO 12), ensuring differential inner wheels never drop below 35% PWM during active turns.

---

## 9. License
This project is licensed under the MIT License.
