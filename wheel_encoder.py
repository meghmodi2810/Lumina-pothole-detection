"""
LUMINA Smart Car - Dual Optical Wheel Encoder Driver (FR13 & Activity Diagram 6)
Monitors left and right optical wheel encoders on GPIO 24 and GPIO 25.
Calculates real-world distance traveled from pulse counts, estimates wheel velocity,
and provides closed-loop speed compensation for slopes and wheel slip.
"""

import math
import time
import threading
import logging

logger = logging.getLogger("WheelEncoder")

# Default Hardware Pins (BCM)
DEFAULT_LEFT_ENCODER_PIN = 24    # Pin 18 on 40-pin header
DEFAULT_RIGHT_ENCODER_PIN = 25   # Pin 22 on 40-pin header

# Physical Dimensions of Lumina-01 Chassis
WHEEL_DIAMETER_M = 0.065         # 65 mm rubber tire diameter
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M  # ~0.2042 m
ENCODER_SLOTS = 20               # 20-hole slotted optical disc
METERS_PER_PULSE = WHEEL_CIRCUMFERENCE_M / float(ENCODER_SLOTS)  # ~0.01021 m per tick


class WheelEncoder:
    def __init__(self, left_pin=DEFAULT_LEFT_ENCODER_PIN, right_pin=DEFAULT_RIGHT_ENCODER_PIN, is_simulation=False):
        self.left_pin = left_pin
        self.right_pin = right_pin
        self.is_simulation = is_simulation
        self.lock = threading.Lock()

        self.left_pulses = 0
        self.right_pulses = 0
        self.last_sample_time = time.time()
        self.last_sample_distance = 0.0
        self.current_speed_kmh = 0.0

        self.is_hardware = False
        if not is_simulation:
            self._init_hardware()

    def _init_hardware(self):
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            # Falling edge interrupt for optocoupler light beam interruption
            GPIO.add_event_detect(self.left_pin, GPIO.FALLING, callback=self._on_left_pulse, bouncetime=5)
            GPIO.add_event_detect(self.right_pin, GPIO.FALLING, callback=self._on_right_pulse, bouncetime=5)
            self.is_hardware = True
            logger.info(f"Wheel encoders active on LEFT=GPIO {self.left_pin}, RIGHT=GPIO {self.right_pin}")
        except ImportError:
            self.is_hardware = False
            self.is_simulation = True
            logger.info("Wheel encoders running in SIMULATION mode (RPi.GPIO not present)")
        except Exception as e:
            self.is_hardware = False
            self.is_simulation = True
            logger.warning(f"Could not bind wheel encoder GPIO pins: {e}. Falling back to simulation.")

    def _on_left_pulse(self, channel):
        with self.lock:
            self.left_pulses += 1

    def _on_right_pulse(self, channel):
        with self.lock:
            self.right_pulses += 1

    def record_simulated_ticks(self, speed_kmh, dt_sec=0.1):
        """Simulates wheel encoder pulses based on vehicle speed."""
        if not self.is_simulation:
            return
        speed_mps = (speed_kmh * 1000.0) / 3600.0
        distance_moved = speed_mps * dt_sec
        ticks = int(distance_moved / METERS_PER_PULSE)
        with self.lock:
            self.left_pulses += ticks
            self.right_pulses += ticks

    def get_total_pulses(self):
        """Returns average count of pulses from left and right encoders."""
        with self.lock:
            return int((self.left_pulses + self.right_pulses) / 2)

    def get_distance_meters(self):
        """Calculates distance in meters from pulse counts (Diagram 6)."""
        pulses = self.get_total_pulses()
        return round(pulses * METERS_PER_PULSE, 2)

    def update_speed(self):
        """Computes current speed in km/h from encoder delta distance."""
        now = time.time()
        dt = now - self.last_sample_time
        if dt < 0.1:
            return self.current_speed_kmh

        current_dist = self.get_distance_meters()
        delta_dist = current_dist - self.last_sample_distance
        if delta_dist < 0:
            delta_dist = 0

        # Speed = (meters / seconds) * 3.6 = km/h
        speed_kmh = (delta_dist / dt) * 3.6
        with self.lock:
            self.current_speed_kmh = round(speed_kmh, 1)
            self.last_sample_time = now
            self.last_sample_distance = current_dist

        return self.current_speed_kmh

    def compute_slope_compensation(self, target_speed_kmh, current_throttle_pwm):
        """
        FR13: Use wheel encoder feedback to adjust motor speed to maintain
        constant speed on slopes and reduce wheel slip.
        """
        actual_speed = self.update_speed()
        if target_speed_kmh <= 0 or current_throttle_pwm <= 0:
            return current_throttle_pwm

        speed_error = target_speed_kmh - actual_speed
        # Proportional trim (clamped between -15% and +15% PWM adjustment)
        pwm_adjustment = max(-15.0, min(15.0, speed_error * 2.5))
        compensated_pwm = max(35.0, min(100.0, current_throttle_pwm + pwm_adjustment))
        return round(compensated_pwm, 1)

    def reset(self):
        """Resets pulse counters to zero at the start of a drive session."""
        with self.lock:
            self.left_pulses = 0
            self.right_pulses = 0
            self.last_sample_distance = 0.0
            self.last_sample_time = time.time()
            self.current_speed_kmh = 0.0
