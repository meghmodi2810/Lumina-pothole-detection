"""
Unit tests for LUMINA Reversed Chassis Mechanics & Autonomous Speed (80%)
Verifies:
1. MotorController pin mapping reversed so current rear (camera/ultrasonic side) is the new front.
2. Directional actions (forward, backward, left, right, left_side_only, right_side_only) map correctly to new front.
3. Proportional drive commands correctly actuate differential speed.
4. AutonomousDriver defaults to 80% cruise speed.
5. Autonomous cruising sends 0.80 throttle.
"""

import unittest
from unittest.mock import MagicMock, patch
from motor_controller import MotorController
from autonomous_driver import AutonomousDriver, AutonomousState, DEFAULT_CRUISE_SPEED


class TestReversedChassisMechanics(unittest.TestCase):
    def setUp(self):
        # Create simulated motor controller
        self.motor = MotorController(
            in1=23, in2=22, ena=12,
            in3=27, in4=17, enb=18,
            default_speed_pct=80.0
        )

    def test_default_pin_mapping_reversed(self):
        """New Front (camera/ultrasonic side): Left=(23,22,12), Right=(27,17,18)"""
        default_motor = MotorController()
        self.assertEqual(default_motor.in1, 23)
        self.assertEqual(default_motor.in2, 22)
        self.assertEqual(default_motor.ena, 12)
        self.assertEqual(default_motor.in3, 27)
        self.assertEqual(default_motor.in4, 17)
        self.assertEqual(default_motor.enb, 18)

    def test_forward_action_sets_correct_states(self):
        """Forward drives towards camera/ultrasonic with 80% power"""
        self.motor.forward(80.0)
        self.assertTrue(self.motor.is_driving)
        self.assertEqual(self.motor.current_action, "forward")
        self.assertEqual(self.motor.current_throttle, 0.8)
        self.assertEqual(self.motor.left_pwm_pct, 80.0)
        self.assertEqual(self.motor.right_pwm_pct, 80.0)

    def test_backward_action_sets_correct_states(self):
        """Backward drives away from camera/ultrasonic with 80% power"""
        self.motor.backward(80.0)
        self.assertTrue(self.motor.is_driving)
        self.assertEqual(self.motor.current_action, "backward")
        self.assertEqual(self.motor.current_throttle, -0.8)
        self.assertEqual(self.motor.left_pwm_pct, 80.0)
        self.assertEqual(self.motor.right_pwm_pct, 80.0)

    def test_left_spin_turn(self):
        """Left spin turn: Left reverse, Right forward (counter-clockwise)"""
        self.motor.left(80.0)
        self.assertTrue(self.motor.is_driving)
        self.assertEqual(self.motor.current_action, "left")
        self.assertEqual(self.motor.current_steering, -0.8)

    def test_right_spin_turn(self):
        """Right spin turn: Left forward, Right reverse (clockwise)"""
        self.motor.right(80.0)
        self.assertTrue(self.motor.is_driving)
        self.assertEqual(self.motor.current_action, "right")
        self.assertEqual(self.motor.current_steering, 0.8)

    def test_left_side_only(self):
        """Pulsing left channel only drives left motors (23,22,12) with right off"""
        self.motor.left_side_only(speed=80.0, forward=True)
        self.assertTrue(self.motor.is_driving)
        self.assertEqual(self.motor.current_action, "left_only")
        self.assertEqual(self.motor.left_pwm_pct, 80.0)
        self.assertEqual(self.motor.right_pwm_pct, 0.0)

    def test_right_side_only(self):
        """Pulsing right channel only drives right motors (27,17,18) with left off"""
        self.motor.right_side_only(speed=80.0, forward=True)
        self.assertTrue(self.motor.is_driving)
        self.assertEqual(self.motor.current_action, "right_only")
        self.assertEqual(self.motor.left_pwm_pct, 0.0)
        self.assertEqual(self.motor.right_pwm_pct, 80.0)

    def test_proportional_drive_80_percent(self):
        """Proportional drive at 80% throttle"""
        self.motor.set_drive_command(steering=0.0, throttle=0.80)
        self.assertTrue(self.motor.is_driving)
        self.assertEqual(self.motor.current_action, "forward")
        self.assertAlmostEqual(self.motor.left_pwm_pct, 80.0, delta=0.5)
        self.assertAlmostEqual(self.motor.right_pwm_pct, 80.0, delta=0.5)


class TestAutonomous80PercentSpeed(unittest.TestCase):
    def test_default_cruise_speed_is_80(self):
        """Verify DEFAULT_CRUISE_SPEED constant is 80.0"""
        self.assertEqual(DEFAULT_CRUISE_SPEED, 80.0)

    def test_driver_initializes_with_80_percent(self):
        """AutonomousDriver defaults to 80% speed"""
        mock_motor = MagicMock()
        driver = AutonomousDriver(motor_controller=mock_motor, is_simulation=True)
        self.assertEqual(driver.cruise_speed, 80.0)
        telemetry = driver.get_telemetry()
        self.assertEqual(telemetry["thresholds"]["cruiseSpeed"], 80.0)


if __name__ == "__main__":
    unittest.main()
