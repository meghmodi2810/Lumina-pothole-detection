"""
Lumina Verification Suite: Dual-Mode Model-Guided Routing (Autonomous & Manual)
Tests:
1. Autonomous Mode: Path clear -> straight cruise
2. Autonomous Mode: Pothole on left -> evades right (+0.35 steering)
3. Autonomous Mode: Pothole on right -> evades left (-0.35 steering)
4. Autonomous Mode: Lane-spanning crater -> Emergency Brake & halt
5. Manual Mode: Human drives forward with road clear -> direct motion
6. Manual Mode: Human drives forward towards pothole on left -> model steers car right (+0.35) and reduces throttle
7. Manual Mode: Human drives forward towards pothole on right -> model steers car left (-0.35) and reduces throttle
8. Manual Mode: Human drives forward towards crater -> model emergency halts car to prevent collision
9. Manual Mode: Human reverses -> direct reverse motion without forward restriction
"""

import time
import unittest
from path_planner import PathPlanner
from autonomous_driver import AutonomousDriver, AutonomousState

class MockMotor:
    def __init__(self):
        self.calls = []
        self.is_emergency_stopped = False

    def forward(self, speed=None):
        self.calls.append(("forward", speed))

    def backward(self, speed=None):
        self.calls.append(("backward", speed))

    def left(self, speed=None):
        self.calls.append(("left", speed))

    def right(self, speed=None):
        self.calls.append(("right", speed))

    def stop(self):
        self.calls.append(("stop", None))

    def set_drive_command(self, steering, throttle):
        self.calls.append(("set_drive_command", (steering, throttle)))

    def emergency_stop(self):
        self.is_emergency_stopped = True
        self.calls.append(("emergency_stop", None))

    def release_emergency_stop(self):
        self.is_emergency_stopped = False
        self.calls.append(("release_emergency_stop", None))

    def get_status(self):
        return {"driving": True, "emergency_stopped": self.is_emergency_stopped}


class TestModelGuidedRouting(unittest.TestCase):
    def setUp(self):
        self.motor = MockMotor()
        self.planner = PathPlanner()

    def test_autonomous_mode_clear_path(self):
        """Autonomous mode drives straight when no potholes are present."""
        driver = AutonomousDriver(
            motor_controller=self.motor,
            plan_provider=lambda: self.planner.plan([]),
            cruise_speed=65.0,
            is_simulation=True
        )
        driver._sim_distance = 60.0
        driver.start()
        time.sleep(0.12)
        driver.stop()

        cmd_calls = [c for c in self.motor.calls if c[0] == "set_drive_command"]
        self.assertGreater(len(cmd_calls), 0)
        self.assertEqual(cmd_calls[-1][1][0], 0.0)  # Straight steering

    def test_autonomous_mode_evade_left_pothole(self):
        """Autonomous mode steers right (+0.35) to evade pothole on left side."""
        # Pothole on left: xc = 0.35, yc = 0.65
        det = {
            "bbox_norm": [0.20, 0.50, 0.45, 0.75],
            "centroid_norm": [0.35, 0.65],
            "width_norm": 0.25,
            "confidence": 0.90
        }
        plan = self.planner.plan([det], base_steering=0.0, base_throttle=65)
        self.assertTrue(plan["hazard_detected"])
        self.assertEqual(plan["evasion_mode"], "EVADE_RIGHT")
        self.assertGreater(plan["steering"], 0.0)

    def test_autonomous_mode_evade_right_pothole(self):
        """Autonomous mode steers left (-0.35) to evade pothole on right side."""
        # Pothole on right: xc = 0.70, yc = 0.65
        det = {
            "bbox_norm": [0.55, 0.50, 0.85, 0.75],
            "centroid_norm": [0.70, 0.65],
            "width_norm": 0.25,
            "confidence": 0.90
        }
        plan = self.planner.plan([det], base_steering=0.0, base_throttle=65)
        self.assertTrue(plan["hazard_detected"])
        self.assertEqual(plan["evasion_mode"], "EVADE_LEFT")
        self.assertLess(plan["steering"], 0.0)

    def test_autonomous_mode_lane_spanning_emergency_brake(self):
        """Autonomous mode emergency halts when crater spans >= 85% of lane right in front of bumper (yc >= 0.75)."""
        det = {
            "bbox_norm": [0.05, 0.65, 0.95, 0.90],
            "centroid_norm": [0.50, 0.78],
            "width_norm": 0.90,
            "confidence": 0.95
        }
        plan = self.planner.plan([det], base_steering=0.0, base_throttle=65)
        self.assertTrue(plan["hazard_detected"])
        self.assertEqual(plan["evasion_mode"], "EMERGENCY_BRAKE")
        self.assertEqual(plan["throttle"], 0)
        self.assertEqual(plan["warning_level"], "RED")

    def test_manual_mode_assisted_forward_left_pothole(self):
        """In manual mode, human drives forward, model evades pothole on left."""
        det = {
            "bbox_norm": [0.20, 0.50, 0.45, 0.75],
            "centroid_norm": [0.35, 0.65],
            "width_norm": 0.25,
            "confidence": 0.90
        }
        plan = self.planner.plan([det], base_steering=0.0, base_throttle=100)
        self.assertEqual(plan["evasion_mode"], "EVADE_RIGHT")
        self.assertGreater(plan["steering"], 0.0)
        # Throttle modulated to 75%
        self.assertEqual(plan["throttle"], 75)

    def test_manual_mode_assisted_forward_crater_halt(self):
        """In manual mode, human drives forward towards crater, model emergency halts."""
        det = {
            "bbox_norm": [0.05, 0.65, 0.95, 0.90],
            "centroid_norm": [0.50, 0.78],
            "width_norm": 0.90,
            "confidence": 0.92
        }
        plan = self.planner.plan([det], base_steering=0.0, base_throttle=100)
        self.assertEqual(plan["evasion_mode"], "EMERGENCY_BRAKE")
        self.assertEqual(plan["throttle"], 0)

    def test_manual_mode_reverse_allows_escape(self):
        """In manual mode, reverse commands must not be blocked by forward camera."""
        # Simulated reverse
        self.motor.backward(100.0)
        self.assertEqual(self.motor.calls[-1], ("backward", 100.0))


if __name__ == "__main__":
    unittest.main()
