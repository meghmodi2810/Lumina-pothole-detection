# LUMINA IoT Smart Car: Autonomous Mode & Hardware Diagnostics Subsystem

> **Artifact Document**: Edge System Integration, Hardware Self-Test, and Closed-Loop Autonomous Navigation  
> **Target System**: Raspberry Pi 4B (Debian GNU/Linux 12 / Bookworm aarch64)  
> **Repository**: `E:\Lumina-pothole-detection-main`  
> **Companion App**: LUMINA Flutter Mobile App (`E:\LUMINA`)

---

## 1. System Overview

This subsystem elevates the LUMINA robotic vehicle from a teleoperated IoT vehicle to a self-sufficient, intelligent autonomous robotic platform. It introduces two core capabilities:

1. **Hardware Startup Self-Test & Diagnostic Engine (`diagnostics_engine.py`)**:
   - Actively checks physical peripherals before driving: MicroSD storage health, HC-SR04 sonar responsiveness, CSI camera frame capture, YOLOv8 segmentation model residency in RAM, and SoC thermal limits.
   - Triggers physical feedback: Pulses the **Green Status LED on GPIO 5 for 3.0 seconds** upon successful self-test.
   - Transmits granular subsystem error diagnostics to the companion app if any component is degraded or disconnected.

2. **Closed-Loop Autonomous Driving Engine (`autonomous_driver.py`)**:
   - Implements a reactive, multi-state navigation loop monitoring surroundings at 15–20 Hz using ultrasonic sonar.
   - Autonomously cruises forward, detects obstacles, performs intelligent non-repetitive evasion (reverse and pivot turn), verifies clearance before resuming, and safely halts if trapped (Situation D) to prevent infinite spinning loops.
   - Employs calibrated hardware PWM on `ENA` (GPIO 18) and `ENB` (GPIO 12) to protect against inrush current brownouts.

---

## 2. Hardware Architecture & Pin Configuration

| Component | Hardware Interface | Raspberry Pi Pin (BCM) | Physical Header Pin | Electrical Characteristics |
| :--- | :--- | :--- | :--- | :--- |
| **HC-SR04 Trigger** | Output | **GPIO 16** | Pin 36 | 15 µs high pulse |
| **HC-SR04 Echo** | Input | **GPIO 26** | Pin 37 | Pulse duration $\propto$ distance (via voltage divider) |
| **Status Green LED** | Output | **GPIO 5** | Pin 29 | 3.3V active HIGH, current-limited via 330Ω |
| **Left Motors (IN1)** | Output | **GPIO 17** | Pin 11 | L298N Channel A Direction Forward |
| **Left Motors (IN2)** | Output | **GPIO 27** | Pin 13 | L298N Channel A Direction Reverse |
| **Left Speed (ENA)** | Hardware PWM | **GPIO 18** | Pin 12 | PWM Channel 0, 100 Hz duty cycle (0–100%) |
| **Right Motors (IN3)** | Output | **GPIO 22** | Pin 15 | L298N Channel B Direction Forward |
| **Right Motors (IN4)** | Output | **GPIO 23** | Pin 16 | L298N Channel B Direction Reverse |
| **Right Speed (ENB)** | Hardware PWM | **GPIO 12** | Pin 32 | PWM Channel 0, 100 Hz duty cycle (0–100%) |
| **CSI Camera** | MIPI CSI-2 | Unicam /dev/media0 | 15-pin Ribbon | Sony IMX219 / OmniVision OV5647 1080p |

---

## 3. Subsystem 1: Startup Hardware Diagnostic Engine

### 3.1 Verification Protocol
When the startup self-test is invoked via `/api/v1/diagnostics/run`, `diagnostics_engine.py` executes five physical hardware verifications:

```
[Startup Self-Test Requested]
        │
        ├── 1. MicroSD Storage Check (shutil.disk_usage)
        │      └── Verified: Free space >= 500 MB (Actual: 5.1 GB free of 14.0 GB)
        │
        ├── 2. Ultrasonic Sonar Ping (GPIO 16 / 26)
        │      └── Verified: Acoustic bounce received (Actual: ~85–140 cm range)
        │
        ├── 3. CSI Camera Module Frame Capture (picamera2 / V4L2)
        │      └── Verified: 640x480 RGB frame successfully ingested @ 30 FPS
        │
        ├── 4. AI Perception Model Residency (YOLOv8 Segmentation)
        │      └── Verified: road_damage_seg_best.pt resident in GPU/RAM
        │
        └── 5. SoC Thermal Sensor (/sys/class/thermal/thermal_zone0/temp)
               └── Verified: Core temp < 80.0°C (Actual: 45–47°C nominal)
        │
        ├── ALL PASS?
        │      ├── YES ──> Glow Green LED on GPIO 5 for 3.0s (Background Thread)
        │      │           Return JSON { "isSuccessful": true, "ledGlowTriggered": true, "results": [...] }
        │      │
        │      └── NO  ──> Keep LED OFF. Return JSON with exact failing component & error string
```

### 3.2 Granular Failure Diagnostic Messages
If any component fails, the engine provides exact root-cause diagnostics to the Flutter UI:
* **Storage Fault**: `"MicroSD storage critically low: only {free_mb} MB available (minimum 500 MB required for telemetry logging)"`
* **Sonar Fault**: `"HC-SR04 Ultrasonic sensor not responding on TRIG=16, ECHO=26: check VCC/GND jumper wiring"`
* **Camera Fault**: `"Camera module not delivering frames: verify CSI ribbon cable connection or lens cap"`
* **AI Model Fault**: `"YOLOv8 perception model failed to load: road_damage_seg_best.pt missing or corrupt"`
* **Thermal Fault**: `"SoC thermal limit exceeded: CPU temperature {temp}°C >= 80.0°C (risk of thermal throttling)"`

---

## 4. Subsystem 2: Closed-Loop Autonomous Driving Engine

### 4.1 State Machine Architecture
`AutonomousDriver` runs in a dedicated thread at 15–20 Hz, executing the closed-loop navigation state machine:

```
                  ┌───────────────┐
                  │     IDLE      │
                  └───────┬───────┘
                          │ Mode Switch to AUTONOMOUS
                          v
                  ┌───────────────┐
     ┌───────────>│   CRUISING    │<───────────────────────────┐
     │            └───────┬───────┘                            │
     │                    │                                    │
     │                    │ Obstacle Detected (<= 28.0 cm)     │
     │                    v                                    │
     │            ┌───────────────┐                            │
     │            │ OBSTACLE_STOP │                            │
     │            └───────┬───────┘                            │
     │                    │                                    │
     │           Distance < 18.0 cm?                           │
     │             ├── YES ──> [BACKING_OFF] (Reverse 0.35s)   │
     │             └── NO  ──┐                                 │
     │                       v                                 │
     │            ┌────────────────────┐                       │
     │            │   DECIDING_TURN    │                       │
     │            └─────────┬──────────┘                       │
     │                      │                                  │
     │            Attempts >= 4?                               │
     │              ├── YES ──> [BLOCKED_WAITING] (Halt)       │
     │              └── NO                                     │
     │                      v                                  │
     │            ┌────────────────────┐                       │
     │            │      TURNING       │                       │
     │            │ (Left / Right Turn)│                       │
     │            └─────────┬──────────┘                       │
     │                      │ Post-turn ping                   │
     │                      v                                  │
     │             Clearance >= 35.0 cm?                       │
     │               ├── YES ──────────────────────────────────┘
     │               └── NO ──> Increment attempt counter, loop back to DECIDING_TURN
     │
     └── Obstacle cleared while in BLOCKED_WAITING (> 40.0 cm)
```

### 4.2 Handling Navigation Situations (AI Vision Routing + Ultrasonic Safety Fusion)
* **Situation A (Clear Path & AI Vision Evasion Routing)**:
  * While front ultrasonic distance exceeds `28.0 cm`, `AutonomousDriver` actively samples the latest real-time road perception plan generated by the YOLOv8 instance segmentation model (`road_damage_seg_best.pt`) and `PathPlanner`.
  * **Pothole on Left**: Model triggers `EVADE_RIGHT` (`steering = +0.35`). Car executes differential steering curving to the right around the defect while maintaining forward motion.
  * **Pothole on Right**: Model triggers `EVADE_LEFT` (`steering = -0.35`). Car executes differential steering curving to the left around the defect.
  * **Lane-Spanning Crater**: Model triggers `EMERGENCY_BRAKE`. Car immediately stops until the road clears.
  * **Open Path**: Model confirms `CENTER_TRACKING`. Car cruises forward at calibrated **65% PWM duty cycle** (`steering = 0.0`).
* **Situation B (Obstacle Detected)**: When ultrasonic clearance drops to $\le 28.0\text{ cm}$ (physical obstacle or wall), the hardware sonar layer asserts emergency priority and all 4 motors immediately halt (`STOP ALL 4 MOTORS`).
* **Situation C (Finding Alternative Path)**:
  * Proximity Back-Off: If clearance is $< 18.0\text{ cm}$, the car reverses backward for $0.35\text{ s}$ at 70% speed to establish a turning radius.
  * Adaptive Direction Decision: Evaluates turn history to avoid fixed repetitive sequences. If a previous right turn failed to clear the obstacle, it attempts left with an expanded exploration angle.
  * Clearance Verification: Averages 3 fresh ultrasonic pings. If clearance $\ge 35.0\text{ cm}$, clears attempt counters and resumes cruise.
* **Situation D (Trapped / All Directions Blocked)**: If 4 consecutive turns fail to find an open path, the car halts in `BLOCKED_WAITING` to prevent infinite spinning or wall scraping. It passively polls the front distance every 1.0s and automatically resumes once the environment opens up ($\ge 40.0\text{ cm}$).
* **Sensor Fault Safeguard**: If 8 consecutive sonar pings fail to return an echo (indicating disconnected sensor), the car halts immediately with `SENSOR_FAULT`.
* **Mode Switching Safety**: Switching from Autonomous to Manual immediately terminates the background loop and asserts an active motor brake.

---

## 5. Subsystem 3: Motor Controller & Brownout Prevention

### 5.1 Power Rail Dynamics & Inrush Spike Protection
During initial testing with standard L298N logic (pulling ENA/ENB permanently HIGH to 100% full battery voltage), abrupt direction reversals (e.g. from backward to pivot spin) created large inrush current spikes ($> 2.5\text{A}$), pulling down the battery rail and triggering a 5V buck-converter brownout reboot on the Raspberry Pi.

### 5.2 Hardware PWM Solution
`motor_controller.py` was refactored with hardware PWM speed modulation on GPIO 18 (`ENA`) and GPIO 12 (`ENB`):
* **Forward Cruise**: Modulated to **65% duty cycle** (steady forward traction, low current draw).
* **Reverse Clearance**: Modulated to **70% duty cycle**.
* **Pivot Turn**: Modulated to **75% duty cycle** (sufficient torque to turn rubber tires on high-friction surfaces without tripping the PMIC).
* **Manual Joystick**: Retains 100% full-speed capability on explicit demand.

---

## 6. API Reference

### 6.1 `POST /api/v1/diagnostics/run`
Executes hardware self-test and pulses the Green LED if healthy.
* **Response (Success)**:
```json
{
  "isSuccessful": true,
  "ledGlowTriggered": true,
  "results": [
    { "subsystem": "STORAGE", "status": "PASS", "details": "MicroSD filesystem healthy: 5.1 GB free of 14.0 GB (36.3% free, 8.3 GB used)" },
    { "subsystem": "ULTRASONIC", "status": "PASS", "details": "HC-SR04 Ultrasonic sensor verified on TRIG=16, ECHO=26: clearance 142.4 cm" },
    { "subsystem": "CAMERA", "status": "PASS", "details": "Pi Camera Module (picamera2) operational: frame captured (640x480 @ 30 FPS)" },
    { "subsystem": "INFERENCE_CORE", "status": "PASS", "details": "YOLOv8 Perception model loaded and ready in memory: road_damage_seg_best.pt (Segmentation)" },
    { "subsystem": "TEMPERATURE", "status": "PASS", "details": "Core temperature nominal: 46.3°C (Safe limit: 80.0°C)" },
    { "subsystem": "MOTOR_DRIVER", "status": "PASS", "details": "L298N 4WD H-Bridge: Hardware GPIO Active" },
    { "subsystem": "COMMUNICATION", "status": "PASS", "details": "Local Wi-Fi Telemetry Link: OK" }
  ],
  "timestamp": 1794294252460
}
```

### 6.2 `POST /api/v1/vehicle/mode`
Switches vehicle operating mode between Manual and Autonomous.
* **Request**:
```json
{ "mode": "autonomous" }
```
* **Response**:
```json
{
  "success": true,
  "mode": "autonomous",
  "isDriving": true,
  "autonomous": {
    "isActive": true,
    "state": "CRUISING",
    "distanceCm": 136.5,
    "avoidanceAttempts": 0,
    "lastTurnDirection": null,
    "thresholds": {
      "stopDistanceCm": 28.0,
      "backoffDistanceCm": 18.0,
      "clearDistanceCm": 35.0,
      "cruiseSpeed": 65.0
    }
  }
}
```

### 6.3 `GET /api/v1/telemetry`
Returns unified real-time vehicle telemetry including autonomous states, motor PWM, ultrasonic distances, and thermal readings.

---

## 7. Verification & Automated Testing

### 7.1 Unit Tests (`test_autonomous_driver.py`)
A comprehensive unit test suite validates the state machine without requiring physical motors:
* `test_initial_state`: Verifies driver initializes in `IDLE` with motors stopped.
* `test_situation_a_cruising`: Verifies steady forward motor actuation when path is clear.
* `test_situation_b_and_c_obstacle_avoidance`: Verifies immediate stop and backoff when obstacle is placed at 15 cm.
* `test_situation_d_trapped_blocked_waiting`: Verifies transition to `BLOCKED_WAITING` without infinite spinning when all paths are blocked.
* `test_turn_direction_intelligence`: Verifies both left and right directions are explored adaptively.
* `test_emergency_stop`: Verifies immediate hardware cutoff and state transition on emergency stop.

**Result**: `Ran 6 tests in 1.137s — OK`

### 7.2 Hardware In-The-Loop Live Test Log
Live execution log on physical car chassis:
```text
2026-11-10 12:23:43 [INFO] [Motor] Autonomous navigation loop entered.
2026-11-10 12:23:43 [INFO] [Motor] >>> LUMINA AUTONOMOUS MODE ACTIVATED <<<
2026-11-10 12:23:43 [INFO] [Motor] [Action] ALL 4 WHEELS FORWARD @ 65% POWER
2026-11-10 12:23:44 [INFO] [Motor] [Obstacle Detected] Clearance 16.9 cm <= 28.0 cm! Stopping.
2026-11-10 12:23:44 [INFO] [Motor] [Action] STOP ALL 4 MOTORS
2026-11-10 12:23:44 [INFO] [Motor] [Proximity Close: 16.9 cm] Reversing 0.35s for turn clearance...
2026-11-10 12:23:44 [INFO] [Motor] [Action] ALL 4 WHEELS BACKWARD @ 70% POWER
2026-11-10 12:23:44 [INFO] [Motor] [Evasion Attempt #1] Turning RIGHT for 0.32s...
2026-11-10 12:23:44 [INFO] [Motor] [Action] 4WD TURN RIGHT @ 75% POWER
2026-11-10 12:23:45 [INFO] [Motor] [Turn Complete] Post-turn clearance: 138.2 cm (required >= 35.0 cm)
2026-11-10 12:23:45 [INFO] [Motor] [Path Clear!] Resuming forward cruise.
2026-11-10 12:23:45 [INFO] [Motor] [Action] ALL 4 WHEELS FORWARD @ 65% POWER
```

---

## 8. Pothole Model False-Positive Elimination & App Hardening (Nov 2026 Overhaul)

### 8.1 Tile Artifact Elimination & Degenerate Box Filtering
- **Problem**: On floor tiles, camera auto-gain and edge shadows triggered low-confidence artifact boxes spanning $>80\%$ of the screen ($x_1=1, x_2=254, y_1=0, y_2=189$). This falsely marked `EMERGENCY_BRAKE` and froze the car indefinitely on tiles.
- **Root Cause**: Downsampling frames to 256x192 without filtering screen-spanning false positives caused YOLOv8 to misinterpret tile boundary contrast as a massive road crater.
- **Solution**:
  1. In `pothole_engine.py`, added strict boundary clamping and degenerate box rejection: any bounding box with width $>80\%$ AND height $>80\%$ of frame dimensions is filtered out as sensor noise.
  2. Rate-limited `_ai_perception_worker` to 12 FPS (`sleep(0.08)`), reducing Raspberry Pi CPU utilization from 224% down to 65–75% and eliminating camera pipeline frame starvation.

### 8.2 Autonomous Deadlock Recovery
- **Problem**: When `PathPlanner` flagged `EMERGENCY_BRAKE`, the car called `self.motor.stop()` but kept `self.state = AutonomousState.CRUISING`, freezing in place with no recovery.
- **Solution**: Updated `autonomous_driver.py` so `EMERGENCY_BRAKE` transitions to `AutonomousState.OBSTACLE_STOP`. The car actively initiates back-off (`BACKING_OFF`), evaluates left/right pivot turns (`DECIDING_TURN`), and escapes dead ends autonomously.

### 8.3 Mobile Companion App Overhaul
- **True Disconnection**: Added `disconnectVehicle()` to `LuminaAppState` and implemented a dedicated **DISCONNECT** button in the top app bar, allowing clean disconnection and return to the Connection screen.
- **Eradication of Mock Storage / 54 GB Simulator Fallback**: Fixed `diagnostics_repository_impl.dart` to use `http.post` with a 12s timeout; removed fake 54.2 GB simulator fallback so real MicroSD metrics (`5.1 GB free of 14.0 GB`) are always displayed.
- **Battery Telemetry Reality**: Removed fake `85%` battery gauge. Exposed real Pi thermal metrics and power rail health.
- **Responsive 4:3 Camera Stream**: Replaced `fit: BoxFit.cover` with `fit: BoxFit.contain` inside `AspectRatio(aspectRatio: 4 / 3)`, ensuring 100% of the live camera view and AR HUD overlays are visible without cropping on any mobile screen.
- **Active Camera Retry**: When tapping RETRY, the app sends a wake-up POST to `/api/v1/camera/start` to ensure the physical camera hardware is running before reconnecting the MJPEG stream.
- **Streamlined Autonomous Cockpit**: In Autonomous Mode, the 4-way D-Pad is hidden, replaced by a large, prominent **STOP AUTONOMOUS DRIVE** button and live intelligence status cards. The D-Pad appears exclusively in Manual Mode.
- **Dynamic Pothole Dialog**: Replaced hardcoded `2.3m AHEAD` mock text with actual detection metadata (Pothole ID, confidence $\%$, and real relative distance).
