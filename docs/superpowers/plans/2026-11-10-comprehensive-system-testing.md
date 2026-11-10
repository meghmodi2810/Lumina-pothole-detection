# LUMINA Comprehensive Hardware, Perception, Autonomy, & Mobile Testing Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute an exhaustive, end-to-end verification and testing battery across the entire LUMINA autonomous vehicle and mobile companion ecosystem, uncovering edge cases, verifying hardware stability, benchmarking perception accuracy, stress-testing under load, and certifying production readiness.

**Architecture:** Multi-tiered test suite spanning physical edge hardware on the Raspberry Pi 4B (`10.40.110.31`), deep unit/integration suites in Python, REST API automated fuzzing, Flutter companion app test suites, and continuous soak/endurance benchmarking.

**Tech Stack:** Python 3.11, PyTorch/Ultralytics (YOLOv8n-Seg), RPi.GPIO, Picamera2, HC-SR04 Sonar, L298N H-Bridge, Flask, Paramiko, Flutter 3.41 / Dart, ADB.

---

### Task 1: Physical Edge Hardware & Sensor Unit Test Suite

**Files:**
- Create: `E:\Lumina-pothole-detection-main\tests\test_physical_hardware.py`
- Test on Edge Pi: `/home/lumina/lumina/tests/test_physical_hardware.py`

- [ ] **Step 1: Write comprehensive hardware diagnostic test script**
  Test HC-SR04 sonar pinging, L298N PWM frequency and duty cycle limits, Picamera2 frame acquisition, disk space verification, and SoC thermal telemetry.
- [ ] **Step 2: Deploy and run test script on Raspberry Pi over SSH**
  Execute via `python3 tests/test_physical_hardware.py` on Pi. Verify 100% pass on physical hardware pins.
- [ ] **Step 3: Analyze and record hardware diagnostic output**

---

### Task 2: AI Perception Engine & Pothole Model Stress Suite

**Files:**
- Create: `E:\Lumina-pothole-detection-main\tests\test_perception_benchmark.py`
- Target: `pothole_engine.py`, `models/road_damage_seg_best.pt`

- [ ] **Step 1: Write perception benchmark test script**
  Test model loading, resolution sweeps (256, 320, 480, 640), confidence thresholds (0.15 to 0.70), perspective distance calculations, false-positive box fusion, temporal track consistency across 50 simulated frames.
- [ ] **Step 2: Run benchmark script locally and on Raspberry Pi**
  Record latency (ms/frame), memory usage, and detection output.
- [ ] **Step 3: Verify zero unhandled exceptions or crashes**

---

### Task 3: Autonomous Driver State Machine & Navigation Edge Case Tests

**Files:**
- Modify/Extend: `E:\Lumina-pothole-detection-main\tests\test_autonomous_driver.py`
- Target: `autonomous_driver.py`, `motor_controller.py`, `path_planner.py`

- [ ] **Step 1: Write exhaustive state machine test cases**
  Test all 9 states: `IDLE`, `CRUISING`, `OBSTACLE_STOP`, `BACKING_OFF`, `DECIDING_TURN`, `TURNING`, `BLOCKED_WAITING`, `SENSOR_FAULT`, `EMERGENCY_STOP`.
  Test stiction breakout pulse assertion, intelligent turn alternating, 5-attempt expanding pivot turns, auto-recovery on clear distance, and resume unblocking.
- [ ] **Step 2: Run local test suite**
  Verify all test cases pass with 0 failures.
- [ ] **Step 3: Deploy to Raspberry Pi and run**

---

### Task 4: Complete REST API Automated Fuzzing & Concurrency Test

**Files:**
- Create: `E:\Lumina-pothole-detection-main\tests\test_api_endpoints.py`
- Target: `app.py` live server at `http://10.40.110.31:5000`

- [ ] **Step 1: Write automated REST API test client**
  Exercise every route: `/api/v1/telemetry`, `/api/v1/vehicle/start`, `/api/v1/vehicle/stop`, `/api/v1/vehicle/resume`, `/api/v1/vehicle/mode`, `/api/v1/control`, `/api/v1/emergency_stop`, `/api/v1/emergency_release`, `/api/v1/camera/start`, `/api/v1/diagnostics/run`, `/clean_feed`, `/video_feed`.
  Fuzz with invalid inputs, rapid-fire bursts (50 requests in parallel), and verify clean HTTP response codes and state synchronization.
- [ ] **Step 2: Execute API test battery against live Raspberry Pi server**
- [ ] **Step 3: Verify server log integrity (no unhandled exceptions)**

---

### Task 5: Mobile App Companion Test Suite & Build Verification

**Files:**
- Target: `E:\LUMINA`
- Test: `test/state_machine_test.dart`, `lib/`

- [ ] **Step 1: Run `flutter analyze`**
  Verify 0 warnings, 0 errors, 0 linter violations.
- [ ] **Step 2: Run `flutter test`**
  Verify all unit, state machine, and repository test suites pass.
- [ ] **Step 3: Verify APK build integrity**
  Ensure debug APK is up-to-date and ready for deployment.

---

### Task 6: Continuous Soak & Endurance Stress Test (5 Minutes)

**Files:**
- Create: `E:\Lumina-pothole-detection-main\tests\test_soak_endurance.py`

- [ ] **Step 1: Write soak test monitor**
  Stream telemetry and perception at 15-20 Hz for 5 continuous minutes while monitoring CPU temp, RAM RSS growth, and frame drop rates.
- [ ] **Step 2: Run soak test against Raspberry Pi**
- [ ] **Step 3: Analyze stability metrics and certify endurance**

---

### Task 7: Documentation, Version Control, & Comprehensive Audit Report

**Files:**
- Modify: `AUTONOMOUS_HARDWARE_INTEGRATION_README.md`
- Modify: `README.md`
- Create: Artifact report in `<appDataDir>\brain\<conversation-id>`

- [ ] **Step 1: Update documentation with complete test audit findings**
- [ ] **Step 2: Commit and push both repositories to `origin/neel` (NEVER `lisa`)**
- [ ] **Step 3: Deliver comprehensive audit report to user**
