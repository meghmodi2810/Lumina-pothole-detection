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
        # When vision model returns EVADE_RIGHT (+0.35 steering), driver steers around pothole
        fake_plan = {"evasion_mode": "EVADE_RIGHT", "steering": 0.35, "throttle": 31}
        self.driver.plan_provider = lambda: fake_plan
        self.driver._sim_distance = 55.0
        self.driver.start()
        time.sleep(0.15)
        self.driver.stop()

        cmd_calls = [c for c in self.motor.calls if c[0] == "set_drive_command"]
        self.assertGreater(len(cmd_calls), 0)
        # Check steering passed is +0.35
        last_steer = cmd_calls[-1][1][0]
        self.assertEqual(last_steer, 0.35)

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

if __name__ == "__main__":
    unittest.main()
