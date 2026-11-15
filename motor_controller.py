"""
LUMINA Autonomous IoT Car - Motor Controller & Hardware Actuator
Differential Drive PWM Controller for L298N / TB6612FNG Dual H-Bridge

Pin Mapping (Raspberry Pi 40-Pin Header BCM) - REVERSED CHASSIS MECHANICS:
  The camera and HC-SR04 ultrasonic sensor are mounted on the physical rear of the chassis,
  which now acts as the vehicle FRONT (primary perception and forward travel direction).
  
  - New Left Motor (facing new camera/ultrasonic front):
      IN1 = GPIO 23 (Pin 16) -> Direction Forward (towards camera)
      IN2 = GPIO 22 (Pin 15) -> Direction Reverse (away from camera)
      ENA = GPIO 12 (Pin 32) -> PWM Speed Control (0-100%)
  - New Right Motor (facing new camera/ultrasonic front):
      IN3 = GPIO 27 (Pin 13) -> Direction Forward (towards camera)
      IN4 = GPIO 17 (Pin 11) -> Direction Reverse (away from camera)
      ENB = GPIO 18 (Pin 12) -> PWM Speed Control (0-100%)
"""

import time
import threading
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] [Motor] %(message)s")
logger = logging.getLogger("MotorController")

GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO_AVAILABLE = True
    logger.info("RPi.GPIO detected. Real hardware actuation enabled.")
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False
    logger.warning("RPi.GPIO not available. Initializing high-fidelity Simulated Motor Driver.")


class MotorController:
    def __init__(
        self,
        in1=23, in2=22, ena=12,
        in3=27, in4=17, enb=18,
        pwm_frequency=100,
        default_speed_pct=100.0,
        min_start_pwm=50,
        max_speed_ceiling=1.0,
        watchdog_timeout_sec=1.5,
        trim_left=1.0,
        trim_right=1.0,
        **kwargs
    ):
        self.in1 = in1
        self.in2 = in2
        self.ena = ena
        self.in3 = in3
        self.in4 = in4
        self.enb = enb
        self.pwm_frequency = pwm_frequency
        self.default_speed_pct = float(default_speed_pct)
        self.min_start_pwm = 60
        self.max_speed_ceiling = 1.0
        self.watchdog_timeout_sec = watchdog_timeout_sec
        self.trim_left = max(0.5, min(1.5, float(trim_left)))
        self.trim_right = max(0.5, min(1.5, float(trim_right)))

        self.is_hardware = GPIO_AVAILABLE
        self.pwm_left = None
        self.pwm_right = None

        self.current_action = "stop"
        self.current_steering = 0.0
        self.current_throttle = 0.0
        self.left_pwm_pct = 0.0
        self.right_pwm_pct = 0.0
        self.is_emergency_stopped = False
        self.is_driving = False
        self.invert_direction = False

        self.last_command_time = time.time()
        self._lock = threading.Lock()
        self._watchdog_active = True

        self._init_gpio()
        self._start_watchdog()

    def set_motor_trim(self, trim_left=1.0, trim_right=1.0):
        """Calibrates motor balance to eliminate left/right mechanical power differences."""
        with self._lock:
            self.trim_left = max(0.5, min(1.5, float(trim_left)))
            self.trim_right = max(0.5, min(1.5, float(trim_right)))
            logger.info(f"[Trim] Motor balance calibrated: Left={self.trim_left:.2f}, Right={self.trim_right:.2f}")

    def get_motor_trim(self) -> dict:
        """Returns current trim multipliers."""
        with self._lock:
            return {"trim_left": self.trim_left, "trim_right": self.trim_right}

    def set_direction_inversion(self, inverted: bool):
        """Inverts front and rear chassis direction (180 degree flip)."""
        with self._lock:
            self.invert_direction = bool(inverted)
            logger.info(f"[Inversion] Chassis direction inverted: {self.invert_direction}")

    def get_direction_inversion(self) -> bool:
        """Returns True if chassis direction is inverted (front <-> rear)."""
        with self._lock:
            return self.invert_direction

    def _init_gpio(self):
        if self.is_hardware:
            try:
                GPIO.setup([self.in1, self.in2, self.in3, self.in4, self.ena, self.enb], GPIO.OUT)
                try:
                    self.pwm_left = GPIO.PWM(self.ena, self.pwm_frequency)
                    self.pwm_right = GPIO.PWM(self.enb, self.pwm_frequency)
                    self.pwm_left.start(0)
                    self.pwm_right.start(0)
                    self._has_hardware_pwm = True
                    logger.info("Hardware PWM enabled on ENA (GPIO 18) and ENB (GPIO 12).")
                except Exception as pe:
                    logger.warning(f"PWM initialization skipped ({pe}), falling back to digital HIGH on enable pins.")
                    self._has_hardware_pwm = False
                    GPIO.output(self.ena, GPIO.LOW)
                    GPIO.output(self.enb, GPIO.LOW)

                self.stop()
                logger.info(f"4WD Hardware pins initialized: Left=(IN1:{self.in1}, IN2:{self.in2}, ENA:{self.ena}), "
                            f"Right=(IN3:{self.in3}, IN4:{self.in4}, ENB:{self.enb})")
            except Exception as e:
                logger.error(f"Failed to initialize RPi.GPIO pins: {e}. Falling back to simulated driver.")
                self.is_hardware = False

    def _start_watchdog(self):
        """Auto-stops motors if mobile connection drops or no commands received."""
        def watchdog_loop():
            while self._watchdog_active:
                time.sleep(0.1)
                with self._lock:
                    if self.is_driving and not self.is_emergency_stopped:
                        elapsed = time.time() - self.last_command_time
                        if elapsed > self.watchdog_timeout_sec:
                            logger.info(f"Watchdog auto-brake after {elapsed:.2f}s inactivity.")
                            self._apply_raw_outputs(0, 0, 0, 0, 0.0, 0.0)
                            self.is_driving = False
                            self.current_action = "stop"
                            self.current_throttle = 0.0
                            self.current_steering = 0.0

        thread = threading.Thread(target=watchdog_loop, daemon=True, name="MotorWatchdog")
        thread.start()

    # -------------------------------------------------------------------------
    # 4-DIRECTIONAL TACTILE CONTROLS (Straight, Reverse, Left, Right, Stop)
    # -------------------------------------------------------------------------
    def forward(self, speed=None):
        """Drives ALL 4 WHEELS straight forward (or backward if inverted) with stiction breakout kickstart."""
        spd = max(0.0, min(100.0, float(speed))) if speed is not None else self.default_speed_pct
        with self._lock:
            self.is_emergency_stopped = False
            self.last_command_time = time.time()
            was_stopped = not self.is_driving
            self.is_driving = True
            self.current_action = "forward" if not self.invert_direction else "forward (inverted)"
            self.current_throttle = spd / 100.0
            self.current_steering = 0.0
            if not self.invert_direction:
                # Normal: IN1=1, IN2=0 (Left Forward) | IN3=1, IN4=0 (Right Forward)
                self._apply_raw_outputs(1, 0, 1, 0, spd, spd, was_stopped=was_stopped)
            else:
                # Inverted: Chassis drives in reverse (physical rear becomes forward)
                self._apply_raw_outputs(0, 1, 0, 1, spd, spd, was_stopped=was_stopped)
            logger.info(f"[Action] ALL 4 WHEELS FORWARD @ {spd:.0f}% POWER (Inverted={self.invert_direction}, Kickstart={was_stopped})")

    def backward(self, speed=None):
        """Drives ALL 4 WHEELS straight backward (or forward if inverted) with stiction breakout kickstart."""
        spd = max(0.0, min(100.0, float(speed))) if speed is not None else self.default_speed_pct
        with self._lock:
            self.is_emergency_stopped = False
            self.last_command_time = time.time()
            was_stopped = not self.is_driving
            self.is_driving = True
            self.current_action = "backward" if not self.invert_direction else "backward (inverted)"
            self.current_throttle = -(spd / 100.0)
            self.current_steering = 0.0
            if not self.invert_direction:
                # Normal: IN1=0, IN2=1 (Left Reverse) | IN3=0, IN4=1 (Right Reverse)
                self._apply_raw_outputs(0, 1, 0, 1, spd, spd, was_stopped=was_stopped)
            else:
                # Inverted: Chassis drives forward
                self._apply_raw_outputs(1, 0, 1, 0, spd, spd, was_stopped=was_stopped)
            logger.info(f"[Action] ALL 4 WHEELS BACKWARD @ {spd:.0f}% POWER (Inverted={self.invert_direction}, Kickstart={was_stopped})")

    def left(self, speed=None):
        """Turn / spin left (swapped if inverted)."""
        spd = max(0.0, min(100.0, float(speed))) if speed is not None else self.default_speed_pct
        with self._lock:
            self.is_emergency_stopped = False
            self.last_command_time = time.time()
            was_stopped = not self.is_driving
            self.is_driving = True
            self.current_action = "left"
            self.current_throttle = 0.0
            self.current_steering = -(spd / 100.0)
            if not self.invert_direction:
                # Spin Turn: Left Reverse (0, 1), Right Forward (1, 0)
                self._apply_raw_outputs(0, 1, 1, 0, spd, spd, was_stopped=was_stopped)
            else:
                # Inverted Spin Turn: Left Forward (1, 0), Right Reverse (0, 1)
                self._apply_raw_outputs(1, 0, 0, 1, spd, spd, was_stopped=was_stopped)
            logger.info(f"[Action] 4WD TURN LEFT @ {spd:.0f}% POWER (Inverted={self.invert_direction}, Kickstart={was_stopped})")

    def right(self, speed=None):
        """Turn / spin right (swapped if inverted)."""
        spd = max(0.0, min(100.0, float(speed))) if speed is not None else self.default_speed_pct
        with self._lock:
            self.is_emergency_stopped = False
            self.last_command_time = time.time()
            was_stopped = not self.is_driving
            self.is_driving = True
            self.current_action = "right"
            self.current_throttle = 0.0
            self.current_steering = spd / 100.0
            if not self.invert_direction:
                # Spin Turn: Left Forward (1, 0), Right Reverse (0, 1)
                self._apply_raw_outputs(1, 0, 0, 1, spd, spd, was_stopped=was_stopped)
            else:
                # Inverted Spin Turn: Left Reverse (0, 1), Right Forward (1, 0)
                self._apply_raw_outputs(0, 1, 1, 0, spd, spd, was_stopped=was_stopped)
            logger.info(f"[Action] 4WD TURN RIGHT @ {spd:.0f}% POWER (Inverted={self.invert_direction}, Kickstart={was_stopped})")

    def stop(self):
        """Stops all motor outputs immediately."""
        with self._lock:
            self.last_command_time = time.time()
            self.is_driving = False
            self.current_action = "stop"
            self.current_throttle = 0.0
            self.current_steering = 0.0
            self._apply_raw_outputs(0, 0, 0, 0, 0.0, 0.0, was_stopped=False)
            logger.info("[Action] STOP ALL 4 MOTORS")

    def left_side_only(self, speed=100.0, forward=True):
        """Runs ONLY the Left Motor Pair (Channel A: OUT1/OUT2, ENA) while Right is completely OFF."""
        spd = max(0.0, min(100.0, float(speed)))
        with self._lock:
            self.is_emergency_stopped = False
            self.last_command_time = time.time()
            was_stopped = not self.is_driving
            self.is_driving = True
            self.current_action = "left_only"
            self.current_throttle = (spd / 100.0) if forward else -(spd / 100.0)
            self.current_steering = -1.0
            in1, in2 = (1, 0) if forward else (0, 1)
            self._apply_raw_outputs(in1, in2, 0, 0, spd, 0.0, was_stopped=was_stopped)
            logger.info(f"[Diagnostic] LEFT MOTORS ONLY @ {spd:.0f}% (Forward={forward})")

    def right_side_only(self, speed=100.0, forward=True):
        """Runs ONLY the Right Motor Pair (Channel B: OUT3/OUT4, ENB) while Left is completely OFF."""
        spd = max(0.0, min(100.0, float(speed)))
        with self._lock:
            self.is_emergency_stopped = False
            self.last_command_time = time.time()
            was_stopped = not self.is_driving
            self.is_driving = True
            self.current_action = "right_only"
            self.current_throttle = (spd / 100.0) if forward else -(spd / 100.0)
            self.current_steering = 1.0
            in3, in4 = (1, 0) if forward else (0, 1)
            self._apply_raw_outputs(0, 0, in3, in4, 0.0, spd, was_stopped=was_stopped)
            logger.info(f"[Diagnostic] RIGHT MOTORS ONLY @ {spd:.0f}% (Forward={forward})")

    def execute_action(self, action: str, speed=None):
        """Convenience method to execute action by name string."""
        act = action.strip().lower()
        if act in ("forward", "up", "fwd", "f"):
            self.forward(speed)
        elif act in ("backward", "down", "back", "rev", "b"):
            self.backward(speed)
        elif act in ("left", "l"):
            self.left(speed)
        elif act in ("right", "r"):
            self.right(speed)
        elif act in ("left_only", "left_side", "lo"):
            self.left_side_only(speed or 100.0)
        elif act in ("right_only", "right_side", "ro"):
            self.right_side_only(speed or 100.0)
        elif act in ("stop", "halt", "brake", "s"):
            self.stop()
        else:
            logger.warning(f"Unknown motor action: '{action}'")

    # -------------------------------------------------------------------------
    # PROPORTIONAL / ANALOG CONTROL (Dual Support)
    # -------------------------------------------------------------------------
    def set_drive_command(self, steering: float, throttle: float):
        """Differential drive from steering (-1 to +1) and throttle (-1 to +1)."""
        with self._lock:
            self.last_command_time = time.time()
            if abs(throttle) > 0.05 or abs(steering) > 0.05:
                self.is_emergency_stopped = False
            elif self.is_emergency_stopped:
                return

            throttle = max(-1.0, min(1.0, float(throttle)))
            steering = max(-1.0, min(1.0, float(steering)))

            # If chassis orientation is inverted (180 degree flip), reverse throttle and steering
            if self.invert_direction:
                throttle = -throttle
                steering = -steering

            self.current_steering = steering
            self.current_throttle = throttle

            if abs(throttle) < 0.05 and abs(steering) < 0.05:
                self.is_driving = False
                self.current_action = "stop"
                self._apply_raw_outputs(0, 0, 0, 0, 0.0, 0.0, was_stopped=False)
                return

            was_stopped = not self.is_driving
            self.is_driving = True

            # Calibrated differential drive speed based on throttle with assertive skid-steer bias
            base_speed = max(65.0, min(100.0, abs(throttle) * 100.0))
            if steering >= 0:
                # Steer Right: Outer (Left) wheels drive, Inner (Right) wheels slow down aggressively
                left_speed = base_speed
                right_speed = max(20.0, base_speed * (1.0 - 1.50 * steering))
            else:
                # Steer Left: Outer (Right) wheels drive, Inner (Left) wheels slow down aggressively
                left_speed = max(20.0, base_speed * (1.0 - 1.50 * abs(steering)))
                right_speed = base_speed

            is_forward = throttle >= 0
            if is_forward:
                self.current_action = "forward"
                self._apply_raw_outputs(1, 0, 1, 0, left_speed, right_speed, was_stopped=was_stopped)
            else:
                self.current_action = "backward"
                self._apply_raw_outputs(0, 1, 0, 1, left_speed, right_speed, was_stopped=was_stopped)

    def _apply_raw_outputs(self, in1_val, in2_val, in3_val, in4_val, left_speed, right_speed, was_stopped=False):
        # Apply motor trim multipliers to calibrate Left/Right motor power differences
        actual_left = max(0.0, min(100.0, float(left_speed) * self.trim_left))
        actual_right = max(0.0, min(100.0, float(right_speed) * self.trim_right))

        self.left_pwm_pct = round(actual_left, 1)
        self.right_pwm_pct = round(actual_right, 1)

        if self.is_hardware:
            try:
                # 1. Directional outputs:
                # Left Motors: IN1, IN2
                GPIO.output(self.in1, GPIO.HIGH if in1_val else GPIO.LOW)
                GPIO.output(self.in2, GPIO.HIGH if in2_val else GPIO.LOW)
                # Right Motors: IN3, IN4
                GPIO.output(self.in3, GPIO.HIGH if in3_val else GPIO.LOW)
                GPIO.output(self.in4, GPIO.HIGH if in4_val else GPIO.LOW)

                # 2. Power enable outputs (Hardware PWM with graceful fallback)
                is_left_active = bool(in1_val or in2_val)
                is_right_active = bool(in3_val or in4_val)

                if getattr(self, '_has_hardware_pwm', False) and self.pwm_left is not None and self.pwm_right is not None:
                    target_l = actual_left if is_left_active else 0.0
                    target_r = actual_right if is_right_active else 0.0
                    self.pwm_left.ChangeDutyCycle(target_l)
                    self.pwm_right.ChangeDutyCycle(target_r)
                else:
                    GPIO.output(self.ena, GPIO.HIGH if is_left_active else GPIO.LOW)
                    GPIO.output(self.enb, GPIO.HIGH if is_right_active else GPIO.LOW)
            except Exception as e:
                logger.error(f"Error applying hardware motor outputs: {e}")
        else:
            logger.debug(f"[SIM] IN=({in1_val},{in2_val},{in3_val},{in4_val}) PWM=({self.left_pwm_pct:.0f}%, {self.right_pwm_pct:.0f}%) Kickstart={was_stopped}")

    def emergency_stop(self):
        with self._lock:
            self.is_emergency_stopped = True
            self.is_driving = False
            self.current_action = "emergency_stop"
            self.current_throttle = 0.0
            self.current_steering = 0.0
            self._apply_raw_outputs(0, 0, 0, 0, 0.0, 0.0)
            logger.critical("EMERGENCY STOP ENGAGED!")

    def release_emergency_stop(self):
        with self._lock:
            self.is_emergency_stopped = False
            self.last_command_time = time.time()
            logger.info("Emergency stop cleared.")

    def get_status(self):
        with self._lock:
            return {
                "is_hardware": self.is_hardware,
                "is_driving": self.is_driving,
                "is_emergency_stopped": self.is_emergency_stopped,
                "current_action": self.current_action,
                "current_steering": self.current_steering,
                "current_throttle": self.current_throttle,
                "left_pwm_pct": self.left_pwm_pct,
                "right_pwm_pct": self.right_pwm_pct,
                "trim_left": self.trim_left,
                "trim_right": self.trim_right,
            }

    def cleanup(self):
        self._watchdog_active = False
        self.stop()
        if self.is_hardware:
            try:
                if self.pwm_left:
                    self.pwm_left.stop()
                if self.pwm_right:
                    self.pwm_right.stop()
                GPIO.cleanup([self.in1, self.in2, self.in3, self.in4, self.ena, self.enb])
            except Exception:
                pass
