"""
LUMINA Automated REST API Validation, Fuzzing & Concurrency Suite
Tests all 12 edge server API endpoints against the live Raspberry Pi server (10.40.110.31:5000).
Includes high-concurrency request bursts, edge-case boundary fuzzing, and telemetry contract verification.
"""

import os
import json
import time
import unittest
import urllib.request
import urllib.error
from concurrent.futures import ThreadPoolExecutor, as_completed

BASE_URL = os.environ.get("LUMINA_HOST", "http://10.40.110.31:5000")


def http_request(path, method="GET", data=None, timeout=5.0):
    url = f"{BASE_URL}{path}"
    body = json.dumps(data).encode("utf-8") if data is not None else None
    headers = {"Content-Type": "application/json"} if data is not None else {}
    req = urllib.request.Request(url, data=body, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            content = resp.read()
            try:
                parsed = json.loads(content.decode("utf-8"))
            except Exception:
                parsed = content
            return resp.status, parsed
    except urllib.error.HTTPError as he:
        err_content = he.read()
        try:
            err_json = json.loads(err_content.decode("utf-8"))
        except Exception:
            err_json = err_content.decode("utf-8")
        return he.code, err_json
    except Exception as ex:
        return 0, str(ex)


class TestLuminaRestApi(unittest.TestCase):

    def test_01_telemetry_schema_and_types(self):
        """Validates /api/v1/telemetry returns 200 with all mandatory fields and valid types."""
        status, data = http_request("/api/v1/telemetry")
        self.assertEqual(status, 200, f"Expected 200, got {status}: {data}")
        self.assertIsInstance(data, dict)

        mandatory_fields = [
            "boardTemperatureC", "motorTemperatureC", "speedKmh", "distanceM",
            "driveMode", "isDriving", "action", "leftPwm", "rightPwm",
            "fps", "warningLevel", "evasionMode", "autonomousState",
            "ultrasonicDistanceCm", "avoidanceAttempts", "totalPotholesDetected",
            "powerSupplyStatus", "gps"
        ]
        for f in mandatory_fields:
            self.assertIn(f, data, f"Missing telemetry field: {f}")

        # Check types
        self.assertIsInstance(data["boardTemperatureC"], (int, float))
        self.assertIsInstance(data["ultrasonicDistanceCm"], (int, float))
        self.assertIn(data["driveMode"], ["manual", "autonomous"])
        self.assertIsInstance(data["isDriving"], bool)
        print(f"\n[Telemetry Verified] Mode: {data['driveMode']} | State: {data['autonomousState']} | CPU: {data['boardTemperatureC']}C | Sonar: {data['ultrasonicDistanceCm']}cm")

    def test_02_camera_start_wake_handshake(self):
        """Verifies POST /api/v1/camera/start wakes up CSI / USB camera pipeline."""
        status, data = http_request("/api/v1/camera/start", method="POST", data={})
        self.assertEqual(status, 200)
        self.assertTrue(data.get("success"))
        self.assertEqual(data.get("status"), "running")
        print(f"\n[Camera Start Verified] Mode: {data.get('mode')}")

    def test_03_vehicle_drive_mode_switching(self):
        """Verifies switching between manual and autonomous modes via /api/v1/vehicle/mode."""
        # Switch to manual
        s1, d1 = http_request("/api/v1/vehicle/mode", method="POST", data={"mode": "manual"})
        self.assertEqual(s1, 200)
        self.assertEqual(d1.get("mode"), "manual")

        # Switch to autonomous
        s2, d2 = http_request("/api/v1/vehicle/mode", method="POST", data={"mode": "autonomous"})
        self.assertEqual(s2, 200)
        self.assertEqual(d2.get("mode"), "autonomous")
        self.assertIn("autonomous", d2)

        # Switch back to manual safely
        s3, d3 = http_request("/api/v1/vehicle/mode", method="POST", data={"mode": "manual"})
        self.assertEqual(s3, 200)
        self.assertEqual(d3.get("mode"), "manual")

        # Reject invalid mode with HTTP 400
        s4, d4 = http_request("/api/v1/vehicle/mode", method="POST", data={"mode": "invalid_warp_speed"})
        self.assertEqual(s4, 400)
        print("\n[Drive Mode Switching Verified] manual <-> autonomous transitions clean.")

    def test_04_vehicle_start_stop_resume_lifecycle(self):
        """Verifies vehicle start, stop, and resume state lifecycle."""
        # Stop
        s1, d1 = http_request("/api/v1/vehicle/stop", method="POST", data={})
        self.assertEqual(s1, 200)
        self.assertEqual(d1.get("state"), "STOPPED")

        # Start
        s2, d2 = http_request("/api/v1/vehicle/start", method="POST", data={})
        self.assertEqual(s2, 200)
        self.assertEqual(d2.get("state"), "DRIVING")

        # Resume
        s3, d3 = http_request("/api/v1/vehicle/resume", method="POST", data={})
        self.assertEqual(s3, 200)
        self.assertEqual(d3.get("state"), "DRIVING")

        # Clean stop
        http_request("/api/v1/vehicle/stop", method="POST", data={})
        print("\n[Vehicle Lifecycle Verified] Start / Stop / Resume transitions clean.")

    def test_05_tactile_discrete_and_analog_controls(self):
        """Tests discrete 4-way commands and analog proportional control with boundary clamping."""
        # 1. Discrete stop
        s1, d1 = http_request("/api/v1/control", method="POST", data={"action": "stop"})
        self.assertEqual(s1, 200)
        self.assertTrue(d1.get("success"))

        # 2. Proportional forward
        s2, d2 = http_request("/api/v1/control", method="POST", data={"control": {"steering": 0.0, "throttle": 0.5}})
        self.assertEqual(s2, 200)

        # 3. Extreme boundary fuzzing: values beyond +/- 1.0 must be safely clamped
        s3, d3 = http_request("/api/v1/control", method="POST", data={"control": {"steering": 999.0, "throttle": -999.0}})
        self.assertEqual(s3, 200)
        # Verify status clamping
        status_data = d3.get("status", {})
        self.assertLessEqual(status_data.get("current_steering", 0.0), 1.0)
        self.assertGreaterEqual(status_data.get("current_throttle", 0.0), -1.0)

        # Halt motors
        http_request("/api/v1/control", method="POST", data={"action": "stop"})
        print("\n[Controls Verified] Discrete actions and analog inputs clamped safely.")

    def test_06_emergency_stop_and_release(self):
        """Verifies hardware emergency stop and clearing."""
        s1, d1 = http_request("/api/v1/emergency_stop", method="POST", data={})
        self.assertEqual(s1, 200)
        self.assertTrue(d1.get("emergency_stop"))

        # In emergency stop, motor status must reflect is_emergency_stopped = True
        _, tel = http_request("/api/v1/telemetry")
        self.assertTrue(tel.get("isEmergencyStopped"))

        # Release emergency stop
        s2, d2 = http_request("/api/v1/emergency_release", method="POST", data={})
        self.assertEqual(s2, 200)
        self.assertFalse(d2.get("emergency_stop"))

        _, tel2 = http_request("/api/v1/telemetry")
        self.assertFalse(tel2.get("isEmergencyStopped"))
        print("\n[E-Stop Verified] Immediate cut-off and clean release verified.")

    def test_07_hardware_diagnostics_suite(self):
        """Verifies full 7-subsystem diagnostics endpoint."""
        s, d = http_request("/api/v1/diagnostics/run", method="POST", data={})
        self.assertEqual(s, 200)
        self.assertTrue(d.get("isSuccessful"))
        results = d.get("results", [])
        self.assertEqual(len(results), 7, f"Expected 7 subsystem results, got {len(results)}")
        for sub in results:
            self.assertEqual(sub["status"], "PASS", f"Subsystem {sub['subsystem']} failed: {sub['details']}")
        print(f"\n[Hardware Diagnostics Verified] 7/7 Physical checks passing. LED Glow: {d.get('ledGlowTriggered')}")

    def test_08_session_pairing_and_claiming(self):
        """Verifies mobile controller handshake endpoints."""
        s1, d1 = http_request("/api/v1/session/pair", method="POST", data={"pairingCode": "123456"})
        self.assertEqual(s1, 200)
        self.assertTrue(d1.get("controllerClaimed"))
        self.assertIn("sessionId", d1)

        s2, d2 = http_request("/api/v1/session/controller/claim", method="POST", data={})
        self.assertEqual(s2, 200)
        self.assertTrue(d2.get("controllerClaimed"))
        print("\n[Session Handshake Verified] Pairing & Controller claiming clean.")

    def test_09_mjpeg_stream_headers(self):
        """Verifies /clean_feed and /video_feed stream valid MJPEG multipart headers."""
        for endpoint in ["/clean_feed", "/video_feed"]:
            url = f"{BASE_URL}{endpoint}"
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=4.0) as resp:
                content_type = resp.headers.get("Content-Type", "")
                self.assertIn("multipart/x-mixed-replace", content_type)
                self.assertIn("boundary=frame", content_type)
                # Read first chunk
                chunk = resp.read(1024)
                self.assertGreater(len(chunk), 0)
                print(f"\n[MJPEG Stream Verified] {endpoint} returned valid multipart stream.")

    def test_10_high_concurrency_burst_stress(self):
        """Dispatches 50 concurrent requests simultaneously to stress-test server concurrency."""
        total_requests = 50
        endpoints = [
            ("/api/v1/telemetry", "GET", None),
            ("/api/v1/control", "POST", {"action": "stop"}),
            ("/api/v1/telemetry", "GET", None),
            ("/api/v1/control", "POST", {"control": {"steering": 0.0, "throttle": 0.0}}),
        ]

        success_count = 0
        latencies = []

        t_start = time.time()
        with ThreadPoolExecutor(max_workers=10) as executor:
            futures = []
            for i in range(total_requests):
                path, method, payload = endpoints[i % len(endpoints)]
                futures.append(executor.submit(http_request, path, method, payload, 5.0))

            for f in as_completed(futures):
                status, _ = f.result()
                if status == 200:
                    success_count += 1

        total_time = time.time() - t_start
        throughput_rps = total_requests / total_time
        print(f"\n[Concurrency Stress Test] Dispatched {total_requests} requests in {total_time:.2f}s ({throughput_rps:.1f} Req/s)")
        print(f"  - Successful Responses (200 OK): {success_count}/{total_requests}")
        self.assertEqual(success_count, total_requests, f"Some requests failed during burst: {success_count}/{total_requests}")


if __name__ == "__main__":
    unittest.main()
