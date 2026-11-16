# Project Lumina: Autonomous Smart Car Perception & Pothole Avoidance System

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![Ultralytics YOLOv8](https://img.shields.io/badge/YOLOv8-Instance_Segmentation-cyan.svg)](https://docs.ultralytics.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

An end-to-end autonomous smart car perception and navigation platform designed for real-time road hazard detection, precise polygon pothole segmentation, temporal hazard tracking, and dynamic autonomous steering avoidance.

---

## 1. System Architecture

Project Lumina operates as a **100% On-Device Edge Cyber-Physical System**:

```
+----------------------------------------------------------------------------------------+
|                             EDGE VEHICLE (Raspberry Pi 4B)                             |
|                                                                                        |
|  +---------------------+   +--------------------+   +-------------------+   +--------------------+  |
|  |    Pi Camera V2     |   |   HC-SR04 Sensor   |   |  Green Status LED |   |   Acoustic Buzzer  |  |
|  | (1080p @ 30 FPS)    |   | (50Hz Safety Loop) |   | (Pin 29 / GPIO 5) |   | (Pin 3 / GPIO 2)   |  |
|  +----------+----------+   +---------+----------+   +---------+---------+   +---------+----------+  |
|             |                        |                        |                       |             |
|             v                        v                        v                       v             |
|  +----------------------------------------------------------------------------------+  |
|  |                 On-Device Perception Core & Web Server (app.py)                   |  |
|  | - Direct YOLOv8n-Seg CPU Inference (~85ms latency)                                |  |
|  | - 50Hz ultrasonic proximity safety cutoff (< 20 cm = Emergency Halt)             |  |
|  | - Differential PWM Motor Control & Autonomous Obstacle Evasion (L298N)            |  |
|  | - Local HTTP & WebSocket API (Port 5000) for Companion App & Hotspot Driving     |  |
|  +----------------------------------------+-----------------------------------------+  |
+-------------------------------------------|--------------------------------------------+
                                            | (Direct Wi-Fi / Local Hotspot)
                                            v
+----------------------------------------------------------------------------------------+
|                      COMPANION DEVICE (iPad / iPhone / Android / Web)                  |
|                                                                                        |
|  - Real-time Low-Latency Cockpit MJPEG Stream & Telemetry HUD                          |
|  - Direct Hardware Control (Touch Pointer Listeners, Keyboard WASD / Spacebar)         |
|  - 100% On-Device Storage for Pothole Observations, GIS Coordinates & Post-Drive Logs  |
+----------------------------------------------------------------------------------------+
```
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

| Condition | Centroid ($X_c, Y_c$) | Action | Steering ($\Delta\theta$) | Throttle Modulation | AR Corridor Color |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Clear Road** | No hazards in corridor | `CENTER_TRACKING` | `0.00` | `80%` Base Cruise | Emerald Green |
| **Approaching Horizon** | $0.20 \le Y_c < 0.45$ (Corridor) | `EVADE_RIGHT` / `EVADE_LEFT` | `+0.50` / `-0.50` (Proactive bias) | `75%` Lookahead cut | Amber Warning |
| **Immediate Hazard Left** | $X_c < 0.50$, $Y_c \ge 0.45$ | `EVADE_RIGHT` | `+0.85` (Assertive veer) | `75%` Evasion throttle | Amber Warning |
| **Immediate Hazard Right** | $X_c \ge 0.50$, $Y_c \ge 0.45$ | `EVADE_LEFT` | `-0.85` (Assertive veer) | `75%` Evasion throttle | Amber Warning |
| **Lane Blockage** | Both sides blocked or $W > 0.85$ | `EMERGENCY_BRAKE` | `0.00` | `0%` Emergency Halt | Crimson Red |

---

## 4. Repository Structure

```
Project Lumina/
│
├── app.py                       # Vision server, MJPEG streaming & FFmpeg H.264 engine
├── autonomous_driver.py         # Multi-state closed-loop autonomous driving engine
├── buzzer_controller.py         # Multi-threaded acoustic horn & alert buzzer engine (GPIO 2 / Pin 3)
├── fix_buzzer_boot_silence.py   # Bootloader clamp utility to permanently silence buzzer at power-on
├── fix_buzzer_boot_silence.sh   # 1-step bash bootloader clamp runner for Raspberry Pi OS
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

## 6. On-Device Raspberry Pi 4B & Hotspot Direct-Connect

### 6.1 Starting the Car Server
On the Raspberry Pi 4B, the car can run as an automatic systemd background service or manually:

1. **Check or Restart the Background Service**:
   ```bash
   sudo systemctl status lumina-car.service
   sudo systemctl restart lumina-car.service
   ```
2. **Or Launch Manually via Terminal**:
   ```bash
   sudo systemctl stop lumina-car.service   # Stop background service if active
   cd /home/lumina/lumina
   git pull origin neel
   source .venv/bin/activate
   python3 app.py
   ```
   The car will bind to `http://0.0.0.0:5000`.

### 6.2 Connecting the Mobile / iPad Companion App
1. Connect your iPad, phone, or laptop to the **same Wi-Fi / mobile hotspot** as the Raspberry Pi.
2. In the LUMINA Companion App (or Web Cockpit), enter the car's IP (e.g. `http://10.78.252.31:5000` or auto-scan).
3. The app establishes a direct, low-latency link with live MJPEG camera streaming, 50Hz ultrasonic proximity monitoring, and instant touch/keyboard drive control.

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

### `POST /api/v1/control/override`
Clears emergency obstacle brake and resets emergency stop state, allowing immediate manual or autonomous driving continuation.


### `POST /api/v1/camera/start`
Actively restarts or wakes up the edge camera pipeline (CSI Picamera2 or V4L2 USB).

### `POST /api/v1/diagnostics/motor/test`
Runs isolated channel diagnostic pulses (`{"channel": "left" | "right" | "both", "duration": 1.5, "speed": 100.0, "direction": "forward" | "backward"}`) to identify dead, weak, or reverse-wired motors.

### `GET /api/v1/potholes` & `GET /api/v1/potholes/snapshot/<pothole_id>`
Returns chronological road hazard inventory with physical metrics and serves full-resolution camera frame JPEG snapshots captured by edge perception during driving.

### UDP Auto-Discovery Beacon (`Port 5005`)
Broadcasts `DISCOVER_LUMINA` over UDP `255.255.255.255:5005`. The edge vehicle discovery daemon instantly replies with IP and port configuration (`{"service": "LUMINA_CAR", "vehicleId": "LUMINA-01", "port": 5000, "status": "HEALTHY"}`), ensuring automatic zero-configuration binding when host DHCP subnets rotate.


---

## 8. Autonomous Navigation & Hardware Diagnostics

For full documentation on the physical edge car architecture, see [AUTONOMOUS_HARDWARE_INTEGRATION_README.md](AUTONOMOUS_HARDWARE_INTEGRATION_README.md).

### 8.1 Hardware Startup Diagnostics (`/api/v1/diagnostics/run`)
- Checks MicroSD storage health (`shutil.disk_usage`), HC-SR04 sonar responsiveness (TRIG=16, ECHO=26), CSI camera capture (`picamera2`), YOLOv8 model residency in RAM, and SoC thermal ceiling (< 80°C).
- On all checks passing, pulses the **Green Status LED on GPIO 5 for 3.0 seconds**.
- Surfaces actionable subsystem failure diagnostics if any sensor or bus is degraded.

### 8.2 Closed-Loop Autonomous Driving & Reversed Chassis Mechanics (`/api/v1/vehicle/mode`)
- **Reversed Chassis Kinematics**: The camera and HC-SR04 ultrasonic sensor are mounted on the physical rear of the car, which serves as the vehicle's functional **FRONT**. Motor channels are mapped accordingly: New Left Motor on GPIO 23 (IN1), GPIO 22 (IN2), GPIO 12 (ENA); New Right Motor on GPIO 27 (IN3), GPIO 17 (IN4), GPIO 18 (ENB).
- **Situation A (Clear Path)**: Cruises forward at calibrated **80% PWM duty cycle** (`DEFAULT_CRUISE_SPEED = 80.0`) with a **300ms Breakaway Kickstart Pulse & Progressive Decay** to overcome static floor friction and vehicle inertia without stalling.
- **Situation B (Obstacle Detected)**: Immediate motor halt when distance $\le 20.0\text{ cm}$.
- **Situation C (Intelligent Evasion)**: Backs up 0.70s at 80% speed if close ($< 15.0\text{ cm}$), adaptively picks turn direction with a 0.70s high-torque pivot turn (85% PWM), avoiding repetitive loops, and verifies clearance $\ge 26.0\text{ cm}$.
- **Situation D (Trapped / Blocked)**: Halts in `BLOCKED_WAITING` after 5 failed turns, resuming automatically when path clears or immediately upon tapping Resume in the mobile app.
- **Brownout Protection & Stiction Buster**: Modulates hardware PWM on `ENA` (GPIO 12) and `ENB` (GPIO 18), ensuring differential inner wheels never drop below 55% PWM during active turns, with an inrush current soft-stagger (50ms left channel lead) to prevent multi-motor surge from dropping battery voltage.

### 8.4 Network Resilience & Dynamic Subnet Discovery
- **Subnet Rotation Mitigation**: Automatically adapts to Android mobile hotspot DHCP randomization across sessions (`10.40.110.x`, `10.43.219.x`, `10.78.252.x`) via active subnet sensing and dual UDP port 5005 discovery.
- **Wi-Fi Power-Save Elimination**: Permanently disables 802.11 power saving (`powersave = 2`) on the Raspberry Pi 4B to eliminate high-jitter micro-sleeps and socket connection drops.

### 8.5 Flutter Mobile & Web Companion Architecture (Universal Dual-Mode Parity)
- **Zero-Deadlock Offline Companion Mode**: Operates fully autonomously even when physical vehicle hardware is powered down or disconnected. Users can explore the entire HMI, launch self-tests, view live dynamic camera streams, drive with virtual joysticks, and simulate pothole detections.
- **Dynamic Cartography & Speed Profiles**: Real-time vehicle GPS breadcrumb tracking on precision dark marine maps (`_LuminaCartographyPainter`) and true velocity timelines with hazard pins (`_TripSpeedProfilePainter`).
- **Authentic Pothole Visualizer & Telemetry Export**: Bundles authentic road damage frames into local assets, dynamically falling back to authentic imagery if offline, with multi-touch zoom, AI bounding polygon overlays, and 1-tap JSON export.
- **Dynamic Energy Accounting**: Calculates authentic battery consumption upon completing a trip session instead of dummy static numbers.
- **Persistent Preferences & Keyboard Driving Controls**: Automatically stores vehicle configurations (speed governor, headlights, sensitivity, and saved IPs) in persistent storage; includes full desktop/web keyboard shortcuts (`W/A/S/D` or arrows to drive, `Space` for emergency stop).
- **Interactive Diagnostic Troubleshooting Guides**: Replaces cryptic error codes with human-readable fault summaries and numbered step-by-step physical troubleshooting instructions (wiring pins, power checks, camera ribbon orientation, and terminal test commands) for every sensor and edge subsystem.
- **Mobile & Web Universal Parity**: The Flutter codebase compiles cleanly and identically for native Android/iOS mobile devices and Web desktop/tablet browsers with responsive layouts (28/28 automated tests passing).

### 8.6 Native Swift/SwiftUI iOS & iPadOS Companion (`E:\lumina_ios`) & `xtool` USB Deployment
- **Dedicated Native Swift/SwiftUI App**: Built specifically for iPhone and iPad in `E:\lumina_ios` with 100% architectural parity with the Flutter app. Includes 4-stage operational flow (Splash -> Connection Manager -> Startup Self-Test -> Cockpit Scaffold), low-latency MJPEG video streaming (`MJPEGStreamer`), virtual analog thumbstick (`VirtualJoystickView`) & tactical 4-way D-Pad (`DirectionalPadView`), keyboard driving controls (`W/A/S/D`, arrows, spacebar), telemetry HUD dials, Dark Marine Cartography (`MapView`), and immediate hardware emergency stop interlocks.
- **Cross-Platform xtool Engine**: Built, codesigned, and packaged directly from Windows via `xtool` in Ubuntu WSL using the Darwin Swift SDK (`arm64-apple-ios`) and Apple Developer ID provisioning (`lisajisaheb@icloud.com`).
- **1-Click Hardware USB Installation**: Automated direct deployment to USB-connected iPads and iPhones via `pymobiledevice3 apps install`, allowing continuous on-device testing and zero-friction updates.
- **Hardware Integration & Diagnostics**: Directly triggers and verifies car hardware: status LED pulse on GPIO 5, 4WD motor drive pulses, HC-SR04 ultrasonic echo monitoring, YOLOv8 inference telemetry, and deep diagnostics.

### 8.7 Fullscreen Landscape Gaming HUD & Dual-Orientation Universal Design
- **Controlled IP Auto-Discovery**: Replaced abrupt auto-connecting with a confirmation modal (`LUMINA CAR DISCOVERED`) that populates the discovered IP in the textbox and lets the operator review latency and choose to connect or edit.
- **iPad Web & Browser Parity**: Added Private Network Access (`Access-Control-Allow-Private-Network: true`) and OPTIONS preflight handling in `pi_car_server/app.py`, origin subnet auto-detection via `Uri.base.host`, and preset chips (`172.20.10.31` for iOS Hotspots, `192.168.43.31` for Android Hotspots, `192.168.1.31` for Wi-Fi).
- **Responsive Dual-Orientation**: Completely refactored self-test action buttons into a responsive 2x2 grid in portrait mode (`CHANGE IP`, `TEST LED`, `TEST MOTORS`, `RE-TEST`), ensuring zero horizontal clipping or hidden buttons on mobile phones without requiring device tilt.
- **Autonomous Cruise Synchronization**: Selecting autonomous mode instantly updates the application state (`isDriving = true`), dynamically toggling between `STOP AUTONOMOUS CRUISE` (warm coral) when driving and `RESUME AUTONOMOUS CRUISE` (solar lime) when halted.
- **Edge-to-Edge Gaming Fullscreen Mode**: 1-tap fullscreen button locks to strict landscape (`SystemUiMode.immersiveSticky`) with 100% viewport camera feed, translucent gaming telemetry HUD (Speed, Distance, Radar, CPU Temp, FPS, Ping, Battery), dedicated Stop / E-Stop buttons in Autonomous mode, and a thumb-friendly virtual 4-way D-Pad in Manual mode.

### 8.8 Interactive Reconnect Feedback & Diagnostic Modal
- **Active Click Feedback**: When tapping `RECONNECT` in the top lost-connection banner or `OfflineStateScreen`, the button immediately changes to a loading state (`CircularProgressIndicator` + `CONNECTING...`), disabling repeated clicks, and emits an informative SnackBar.
- **Diagnostic Troubleshooting Modal (`showReconnectFailedDialog`)**: If the car is still offline (powered off, wrong IP, or Wi-Fi dropped), an alert modal displays the target IP, connection error details, a 3-point troubleshooting checklist (power, Wi-Fi network match, and IP reassignment), and direct `[TRY AGAIN]` / `[CHANGE IP / SCAN]` / `[DISMISS]` buttons.

### 8.9 Zero-Latency Motor Actuation, Balanced Power & Perception Resilience
- **Zero-Latency Motor Drive (< 5ms)**: Removed all blocking sleep delays from motor actuation. Driving commands apply directly to PWM duty cycle registers instantaneously, eliminating thread lock contention.
- **Equal Motor Power & Trim API (`/api/v1/vehicle/trim`)**: Drives left and right motor channels symmetrically with 100% full power in manual mode and calibrated 80% speed in autonomous mode. Software trim allows fine-tuning mechanical balance.
- **Perception Worker Crash Containment & Global State Fix**: Resolved `UnboundLocalError` on detection arrays and added missing `random` dependency. Restored 0.25 confidence threshold for reliable real-time pothole identification, with 1.5s automatic staleness clearing.
- **Diagnostic Warning Elimination**: Streamlined `check_motor_power` in `diagnostics_engine.py` to prevent false startup battery warnings on unwired pins.

### 8.10 Resilient Telemetry Link & Autonomous Model-Based Pothole Evasion
- **Telemetry Link Hardening**: Eliminated overlapping polling and increased request timeout to 3000ms with a 5.0-second silence threshold in `vehicle_live_source.dart`, resolving frequent 5-second disconnect alerts over Wi-Fi hotspot.
- **Model-Based Autonomous Pothole Evasion**: Integrated real-time YOLOv8 `PathPlanner` steering offsets ($\pm 0.55$) with skid-steer differential drive curves ($1.50 \times \text{steering}$) in `autonomous_driver.py`, allowing the car to actively steer around detected potholes at 80% cruise speed and automatically straighten out once cleared.

### 8.11 Real Slippy Cartography, Persistent Missions & Cockpit Control Unblocking
- **Real Slippy Map Tile Cartography (`RealMapTileView`)**: Replaced placeholder vector grid with high-performance Web Mercator raster tiles from CartoDB Dark Matter and OpenStreetMap. Features interactive pan/drag with inertia, multi-level zoom (12x–18x), live vehicle location marker with heading needle and pulsing radar ring, GPS route breadcrumbs, severity-coded pothole markers, and zero-bandwidth offline fallback.
- **Persistent Real Missions Archive**: Complete JSON serialization and deserialization for Trips, RoutePoints, and PotholeObservations backed by device storage (`SharedPreferences`). Removed fake mock seed trips; real missions are logged with duration, distance, velocity, battery drain, and pothole links, with 1-tap GeoJSON/JSON export and trip deletion capabilities.
- **Cockpit Safety Unblocking**: Relocated pothole detection notifications from the lower steering zone (`bottom: 16`) to the top HUD (`top: 52` / `top: 48`) with 5-second auto-dismissal, ensuring manual steering buttons (D-Pad, Virtual Joystick, WASD) remain 100% accessible and unhindered during driving.
- **Orientation Lag & Freeze Elimination**: Replaced quadratic memory scanning in MJPEG chunk receiver with windowed byte-boundary scanning and static frame caching, eliminating main-isolate thread lockup and blackouts when rotating between landscape and portrait.

### 8.12 Autonomous Steering Authority & Perception Latency Overhaul
- **Early Lookahead Horizon Expansion**: Lowered the PathPlanner reaction threshold from `0.45` to `0.20`. Hazards are recognized 1.5–2.5 meters ahead, triggering early lookahead deceleration to double available reaction time.
- **Skid-Steer Counter-Rotating Pivot Drive**: Overhauled `motor_controller.py` differential drive to eliminate 4WD tire scrub. For steering demand $|\text{steering}| > 0.45$, inner wheels actively counter-rotate (reverse) while outer wheels drive forward, producing sharp, high-torque yaw deflection.
- **Evasion Hold Hysteresis**: Added `min_evasion_hold_sec = 0.40` in `autonomous_driver.py` so that once an evasion is triggered, the trajectory is held for a minimum of 400ms through transient frame drops or camera angle changes, preventing twitch-steering.
- **Zero-Latency Video Queue & Worker Optimization**: Added `cv2.CAP_PROP_BUFFERSIZE = 1` and trimmed perception loop sleep to `0.005s` in `app.py`, eliminating frame queue lag so steering commands act on instantaneous real-time video.

### 8.13 Phone GNSS Injection, PDF Road Survey Reports & Cardboard Motor Trim
- **Phone GNSS Location Injection (`DeviceGpsService`)**: Integrates real mobile device GPS via `geolocator`. Injects real-world street latitude and longitude into telemetry breadcrumbs and pothole observations, solving the lack of onboard car GPS hardware.
- **Official PDF Road Inspection Survey (`TripPdfReportService`)**: Generates print/share-ready vector PDF survey documents with road quality scoring (Grade A/B/C), mission summary statistics, itemized hazard inventory table, and maintenance recommendations.
- **Cardboard Track & Live Motor Trim Calibration (`/api/v1/vehicle/calibration`)**: Added live tuning sliders in the Settings screen for Left/Right motor power trim (0.70x to 1.30x), autonomous cruise speed (40% to 100%), and pothole detection confidence threshold (15% to 65%), along with a 1.2s kickstart straight drive test button to eliminate drift on low-friction cardboard surfaces.

### 8.14 Active Acoustic Signaling & Tactical Horn Engine (`buzzer_controller.py`)
- **NPN Transistor Driver on GPIO 21 (Pin 40)**: Low-side NPN driver switching a 5V active buzzer. Explicitly initializes `GPIO.setup(21, GPIO.OUT, initial=GPIO.LOW)` to prevent spurious humming caused by the I2C1 physical pull-up on GPIO 2.
- **Non-Blocking Acoustic Engine**: Thread-safe background daemon worker thread (`BuzzerAudioThread`) and task queue ensures zero frame drops in the real-time YOLOv8 15–20 FPS perception loop and zero motor control jitter.
- **Intelligent Acoustic Signatures**:
  - **Pothole Detected**: Distinctive double-chirp (80ms ON, 50ms OFF, 80ms ON) alerts driver the instant a road defect is cataloged.
  - **Urgent Obstacle Warning**: 3 rapid warning bursts (90ms ON, 50ms OFF x 3) when front clearance drops below safety stopping threshold.
  - **Tactical Horn**: Supports 250ms pulse or continuous hold-to-honk via `/api/v1/vehicle/horn` and `/api/v1/control` (`action: "horn"`).
  - **System Startup Chime**: Ascending 3-tone arpeggio (60ms, 70ms, 120ms) confirming system readiness on server boot.
  - **Self-Test Pass Chime**: Double confirmation chime synchronized with Green Status LED glow on GPIO 5.
- **Companion App Cockpit Integration**: Prominent tactile HORN buttons in `DriveScreen` top bar, `DriveManualView`, and `FullscreenDriveView` (both manual and autonomous modes) with haptic impact feedback and solar-lime glow, plus a `TEST BUZZER` button in `DiagnosticsScreen`.

### 8.15 Offline Tactical Metric Cartography & Dynamic Trip Lifecycle System
- **100% Offline Tactical Metric Radar (`RealMapTileView`)**: Solved the blank/coordinate tile bug when connected to the vehicle hotspot without cellular data. Renders mathematically accurate concentric radar range rings ($10\text{m}$, $25\text{m}$, $50\text{m}$, $100\text{m}$, $200\text{m}$), orientation compass rose, Mercator-calibrated $50\text{m}$ ground scale bar, pulsing vehicle heading cone, and severity-coded hazard pins.
- **Zero-Flicker Cartography Screen (`MapScreen`)**: Eliminated render thrashing by replacing the in-build `FutureBuilder` with cached observations and dynamic state change listeners. Added live trip recording status pill and quick `+ TEST HAZARD` inspection trigger.
- **Dynamic Trip Lifecycle Engine**:
  - Automatically initializes a mission upon vehicle movement in Manual or Autonomous mode, or manually via `START TRIP`.
  - Prominent cybernetic HUD ribbon on Cockpit HUD: `🔴 REC [ 02:15 ] • [ 86m ] • [ 2 HAZARDS ]` with an interactive `FINISH & SAVE TRIP` button.
  - Finalizes and saves complete telemetry (route polyline, average velocity, battery drain, observed potholes) to local storage.
  - 1-tap direct PDF report export on every trip card in the archive (`TripPdfReportService`).
  - Integrated `createDemoTrip` benchmark generator allowing immediate testing of survey reports and PDF printouts without driving.

### 8.16 Landscape Orientation Stabilization & Stream Reconnection Architecture
- **Camera Stream Persistence (`CameraStreamView`)**: Eliminated app freezing during physical phone rotation or landscape mode conversion. Converted the MJPEG receiver to use a shared broadcast connection manager (`StreamController<Uint8List>.broadcast()`). Rotating the device preserves the active TCP stream at 30 FPS with zero socket thrashing, zero broken pipes on the Flask server, and zero reconnect blackout.
- **Zero-Overflow Responsive Geometry**:
  - `CockpitScaffold`: Scaled the right-aligned status cluster using `Flexible` and `FittedBox` to prevent horizontal overflow on landscape smartphone viewports.
  - `DriveScreen`: Clustered toolbar controls and wrapped mission recording telemetry in flexible text wrappers to prevent sub-pixel overflows during orientation transitions.
  - `DriveManualView`: Constrained the D-pad header row inside flex-3 columns with responsive scaling.
  - `TripsScreen`: Made the empty state placeholder scrollable, resolving a 221px vertical overflow on landscape screens.
- **Comprehensive Orientation Test Coverage**: Automated test suite (`orientation_switch_test.dart`) verifying seamless orientation switching across all five app tabs and multiple device screen resolutions ($640\times 320$, $740\times 360$, $780\times 360$).

### 8.3 Automated Verification & Soak Testing Suite
Comprehensive automated testing is provided in `tests/`:
- `python tests/test_physical_hardware.py`: Runs 7/7 physical sensor, bus, and GPIO diagnostics on the Pi.
- `python tests/test_perception_benchmark.py`: Benchmarks PyTorch inference, box fusion clustering, and temporal tracking.
- `python -m unittest test_autonomous_driver.py test_model_guided_driving.py`: Validates obstacle state machine and trajectory planning.
- `python tests/test_api_endpoints.py`: Validates REST API schemas and high-concurrency request handling.
- `python tests/test_soak_endurance.py`: Multi-minute sustained telemetry soak and thermal monitoring.

---

## 9. License
This project is licensed under the MIT License.
