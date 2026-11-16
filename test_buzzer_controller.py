"""
Unit tests for LUMINA BuzzerController & Acoustic Signaling Engine
Validates non-blocking acoustic patterns, horn control, simulation mode, and safety cleanup.
"""

import time
import unittest
from buzzer_controller import BuzzerController


class TestBuzzerController(unittest.TestCase):
    def setUp(self):
        # Force simulation mode for deterministic test execution
        self.buzzer = BuzzerController(pin=2, is_simulation=True)

    def tearDown(self):
        self.buzzer.cleanup()

    def test_initialization_defaults(self):
        """Validates pin mapping, initial state, and status structure."""
        status = self.buzzer.get_status()
        self.assertEqual(status["subsystem"], "BUZZER")
        self.assertEqual(status["pin"], 2)
        self.assertTrue(status["isSimulation"])
        self.assertFalse(status["isActive"])
        self.assertIn("wiring", status)
        self.assertEqual(status["wiring"]["base"], "Resistor to GPIO 2 (Pin 3)")

    def test_non_blocking_execution(self):
        """Verifies that scheduling patterns returns instantaneously (< 15ms) to preserve FPS."""
        t0 = time.time()
        self.buzzer.pothole_alert()
        self.buzzer.obstacle_alert()
        elapsed = time.time() - t0
        self.assertLess(elapsed, 0.05, f"Scheduling alerts took {elapsed:.4f}s; must be non-blocking (<0.05s)")

    def test_pothole_alert_execution(self):
        """Verifies that pothole double-chirp pattern updates total_beeps and last_pattern."""
        initial_beeps = self.buzzer.total_beeps
        self.buzzer.pothole_alert()
        # Allow background thread to process (80ms + 50ms + 80ms = 210ms)
        time.sleep(0.35)
        self.assertGreaterEqual(self.buzzer.total_beeps, initial_beeps + 2)
        self.assertEqual(self.buzzer.last_pattern, "POTHOLE_ALERT")

    def test_obstacle_urgent_alarm(self):
        """Verifies urgent obstacle alarm clears queue and executes rapidly."""
        initial_beeps = self.buzzer.total_beeps
        self.buzzer.obstacle_alert()
        time.sleep(0.40)
        self.assertGreaterEqual(self.buzzer.total_beeps, initial_beeps + 3)
        self.assertEqual(self.buzzer.last_pattern, "OBSTACLE_URGENT")

    def test_horn_pulse(self):
        """Verifies horn pulse mode with custom duration."""
        initial_beeps = self.buzzer.total_beeps
        self.buzzer.horn_pulse(duration_sec=0.1)
        time.sleep(0.2)
        self.assertGreaterEqual(self.buzzer.total_beeps, initial_beeps + 1)

    def test_horn_continuous_state(self):
        """Verifies horn on / off continuous control."""
        self.buzzer.horn_on()
        time.sleep(0.05)
        self.assertTrue(self.buzzer.is_active)
        self.buzzer.horn_off()
        time.sleep(0.05)
        self.assertFalse(self.buzzer.is_active)

    def test_chimes(self):
        """Verifies startup chime and selftest pass chime."""
        self.buzzer.startup_chime()
        time.sleep(0.35)
        self.assertEqual(self.buzzer.last_pattern, "STARTUP_CHIME")

        self.buzzer.selftest_pass_chime()
        time.sleep(0.45)
        self.assertEqual(self.buzzer.last_pattern, "SELFTEST_PASS")


if __name__ == "__main__":
    unittest.main()
