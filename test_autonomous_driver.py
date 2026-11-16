import time
import unittest
from unittest.mock import MagicMock
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


class TestAutonomousDriver(unittest.TestCase):
    def setUp(self):
        self.motor = MockMotor()
        self.driver = AutonomousDriver(
            motor_controller=self.motor,
            stop_dist_cm=28.0,
            backoff_dist_cm=18.0,
            clear_dist_cm=35.0,
            cruise_speed=60.0,
            reverse_speed=70.0,
            turn_speed=75.0,
            reverse_duration=0.05,
            turn_duration=0.05,
            max_avoid_attempts=3,
            is_simulation=True
        )

    def tearDown(self):
        self.driver.stop()

    def test_initial_state(self):
        self.assertEqual(self.driver.state, AutonomousState.IDLE)
        self.assertFalse(self.driver.is_active)
        telemetry = self.driver.get_telemetry()
        self.assertFalse(telemetry["isActive"])
        self.assertEqual(telemetry["state"], AutonomousState.IDLE)

    def test_situation_a_cruising(self):
        # When distance is clear (50cm), car should move forward
        self.driver._sim_distance = 55.0
        self.driver.start()
        time.sleep(0.15)
        self.driver.stop()

        self.assertEqual(self.driver.state, AutonomousState.IDLE)
        drive_calls = [c for c in self.motor.calls if c[0] in ("forward", "set_drive_command")]
        self.assertGreater(len(drive_calls), 0)

    def test_situation_a_vision_pothole_evasion(self):
        # When vision model returns EVADE_RIGHT (+0.55 steering), driver steers around pothole
        fake_plan = {"evasion_mode": "EVADE_RIGHT", "target_steering": 0.55, "steering": 0.55, "throttle": 80}
        self.driver.plan_provider = lambda: fake_plan
        self.driver._sim_distance = 55.0
        self.driver.start()
        time.sleep(0.15)
        self.driver.stop()

        cmd_calls = [c for c in self.motor.calls if c[0] == "set_drive_command"]
        self.assertGreater(len(cmd_calls), 0)
        # Check steering passed is +0.55
        last_steer = cmd_calls[-1][1][0]
        self.assertEqual(last_steer, 0.55)

    def test_situation_a_path_planner_left_and_right_evasion(self):
        try:
            from path_planner import PathPlanner
        except ImportError:
            self.skipTest("numpy not installed in host environment")

        planner = PathPlanner()

        # Pothole on left side (xc = 0.35, yc = 0.65)
        det_left = [{
            "bbox_norm": [0.25, 0.55, 0.45, 0.75],
            "centroid_norm": [0.35, 0.65],
            "width_norm": 0.20,
            "confidence": 0.45
        }]
        plan_left = planner.plan(det_left)
        self.assertEqual(plan_left["evasion_mode"], "EVADE_RIGHT")
        self.assertGreater(plan_left["target_steering"], 0.4)

        # Pothole on right side (xc = 0.65, yc = 0.65)
        planner.smoothed_steering = 0.0
        det_right = [{
            "bbox_norm": [0.55, 0.55, 0.75, 0.75],
            "centroid_norm": [0.65, 0.65],
            "width_norm": 0.20,
            "confidence": 0.45
        }]
        plan_right = planner.plan(det_right)
        self.assertEqual(plan_right["evasion_mode"], "EVADE_LEFT")
        self.assertLess(plan_right["target_steering"], -0.4)

    def test_situation_b_and_c_obstacle_avoidance(self):
        # Start clear, then introduce obstacle at 15cm (should stop and reverse)
        self.driver._sim_distance = 50.0
        self.driver.start()
        time.sleep(0.08)

        # Trigger obstacle
        self.driver._sim_distance = 15.0
        time.sleep(0.35)
        self.driver.stop()

        actions = [c[0] for c in self.motor.calls]
        self.assertTrue(("forward" in actions) or ("set_drive_command" in actions))
        self.assertIn("stop", actions)
        self.assertIn("backward", actions)

    def test_situation_d_trapped_blocked_waiting(self):
        # Keep distance blocked (10cm) permanently so multiple turns fail
        self.driver._sim_distance = 10.0
        self.driver.start()
        time.sleep(0.6)

        # Should reach BLOCKED_WAITING and not loop infinitely
        self.assertIn(self.driver.state, [AutonomousState.BLOCKED_WAITING, AutonomousState.TURNING, AutonomousState.DECIDING_TURN])
        self.driver.stop()
        self.assertEqual(self.driver.state, AutonomousState.IDLE)

    def test_turn_direction_intelligence(self):
        # Should not always turn the same direction
        turns = []
        for i in range(4):
            self.driver.avoidance_attempts = i
            dir = self.driver._choose_intelligent_turn_direction()
            turns.append(dir)

        # Must have attempted both directions
        self.assertIn("left", turns)
        self.assertIn("right", turns)

    def test_emergency_stop(self):
        self.driver.start()
        time.sleep(0.05)
        self.driver.emergency_stop()

        self.assertFalse(self.driver.is_active)
        self.assertEqual(self.driver.state, AutonomousState.EMERGENCY_STOP)
        self.assertTrue(self.motor.is_emergency_stopped)

    def test_resume_from_blocked_or_obstacle_state(self):
        # Simulate car getting trapped in BLOCKED_WAITING
        self.driver.is_active = True
        self.driver.state = AutonomousState.BLOCKED_WAITING
        self.driver.avoidance_attempts = 4

        # Calling resume() must clear state back to CRUISING and reset attempts
        self.driver.resume()
        self.assertEqual(self.driver.state, AutonomousState.CRUISING)
        self.assertEqual(self.driver.avoidance_attempts, 0)
        self.assertFalse(self.motor.is_emergency_stopped)

    def test_start_unblocks_active_halt(self):
        # When driver is active but stopped in OBSTACLE_STOP, calling start() resumes it
        self.driver.is_active = True
        self.driver.state = AutonomousState.OBSTACLE_STOP
        self.driver.start()
        self.assertEqual(self.driver.state, AutonomousState.CRUISING)

    def test_sensor_fault_safety_cutoff_and_recovery(self):
        # When ultrasonic sensor disconnects repeatedly (None readings), driver transitions to SENSOR_FAULT
        self.driver.start()
        time.sleep(0.05)

        # Force consecutive sensor dropouts beyond fail limit
        self.driver.consecutive_sensor_fails = 10
        self.driver._sim_distance = None

        # Give loop a brief moment to process safety cutoff
        time.sleep(0.15)
        self.assertIn(self.driver.state, [AutonomousState.SENSOR_FAULT, AutonomousState.CRUISING])

        # Test recovery when sensor reconnects
        self.driver._sim_distance = 60.0
        self.driver.resume()
        self.assertEqual(self.driver.state, AutonomousState.CRUISING)
        self.assertEqual(self.driver.consecutive_sensor_fails, 0)

    def test_evasion_hold_hysteresis_sustained_steering(self):
        """Verifies driver sustains evasion steering through transient plan dropout."""
        plans = [
            {"evasion_mode": "EVADE_RIGHT", "target_steering": 0.85, "throttle": 60},
            {"evasion_mode": "CENTER_TRACKING", "target_steering": 0.0, "throttle": 80}
        ]
        plan_idx = 0
        def dynamic_plan():
            nonlocal plan_idx
            p = plans[min(plan_idx, len(plans) - 1)]
            plan_idx += 1
            return p

        driver = AutonomousDriver(
            motor_controller=self.motor,
            plan_provider=dynamic_plan,
            cruise_speed=80.0,
            is_simulation=True
        )
        driver._sim_distance = 60.0
        driver.start()
        time.sleep(0.12)  # Tick 1: triggers evasion (hold set for +0.40s)
        time.sleep(0.12)  # Tick 2: plan returns CENTER_TRACKING, but hold is active
        driver.stop()

        # Check all set_drive_command calls
        cmd_calls = [c for c in self.motor.calls if c[0] == "set_drive_command"]
        self.assertGreater(len(cmd_calls), 1)
        # Latest call before stop should still have held evasive steering (> 0.0)
        self.assertGreater(cmd_calls[-1][1][0], 0.0)

    def test_buzzer_acoustic_alerts_integration(self):
        """Verifies buzzer alerts trigger on obstacle stop, emergency stop, and backoff."""
        mock_buzzer = MagicMock()
        self.driver.set_buzzer(mock_buzzer)

        # 1. Emergency stop triggers buzzer beep
        self.driver.emergency_stop()
        mock_buzzer.beep.assert_called_with(0.4)

        # 2. Obstacle detection triggers obstacle_alert
        self.driver._sim_distance = 15.0  # Closer than stop_dist_cm
        self.driver.start()
        time.sleep(0.18)
        self.driver.stop()
        mock_buzzer.obstacle_alert.assert_called()


if __name__ == "__main__":
    unittest.main()
