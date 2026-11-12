"""
LUMINA Autonomous IoT Vehicle - Autonomous Driving & Intelligent Obstacle Avoidance Engine
Implements adaptive closed-loop navigation using HC-SR04 Ultrasonic Sonar and 4WD Motor Actuation.

Behaviors:
- Situation A: Clear Path -> Move forward steadily, continuously measuring front clearance.
- Situation B: Obstacle Detected -> Stop immediately to prevent collision.
- Situation C: Finding Alternative Path -> Reverse slightly if close, intelligently choose turn
              direction (avoiding fixed repetitive patterns), probe new angle, verify clearance.
- Situation D: Path Blocked in Every Attempt -> Safely stop, avoid infinite loop/spin, wait for path change.
- Safety: Auto-stops if ultrasonic sensor disconnects; instant cut on mode switch or E-Stop.
"""

import time
import random
import logging
import threading
from collections import deque

logger = logging.getLogger("AutonomousDriver")

# Hardware Pins
DEFAULT_TRIG_PIN = 16
DEFAULT_ECHO_PIN = 26

# Navigation Thresholds (Calibrated for Real Chassis Physics & Stiction)
DEFAULT_STOP_DISTANCE_CM = 20.0       # Stop if obstacle closer than 20cm
DEFAULT_BACKOFF_DISTANCE_CM = 15.0    # Reverse slightly if closer than 15cm
DEFAULT_CLEAR_DISTANCE_CM = 26.0      # Path considered clear to cruise
DEFAULT_CRUISE_SPEED = 95.0           # High-torque forward cruise speed (prevents low-voltage motor stall)
DEFAULT_REVERSE_SPEED = 98.0          # High-torque reverse
DEFAULT_TURN_SPEED = 98.0             # High-torque skid steer turn
DEFAULT_REVERSE_DURATION = 0.70       # Full 0.70s reverse (~12cm clearance)
DEFAULT_TURN_PULSE_DURATION = 0.70    # Full 0.70s pivot turn (~45-60 deg)
DEFAULT_MAX_AVOID_ATTEMPTS = 5        # Max failed turn attempts before halting in blocked state
DEFAULT_SENSOR_FAIL_LIMIT = 8         # Consecutive hardware disconnects before safety stop


class AutonomousState:
    IDLE = "IDLE"
    CRUISING = "CRUISING"
    OBSTACLE_STOP = "OBSTACLE_STOP"
    BACKING_OFF = "BACKING_OFF"
    DECIDING_TURN = "DECIDING_TURN"
    TURNING = "TURNING"
    BLOCKED_WAITING = "BLOCKED_WAITING"
    SENSOR_FAULT = "SENSOR_FAULT"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class AutonomousDriver:
    def __init__(
        self,
        motor_controller,
        plan_provider=None,
        trig_pin=DEFAULT_TRIG_PIN,
        echo_pin=DEFAULT_ECHO_PIN,
        stop_dist_cm=DEFAULT_STOP_DISTANCE_CM,
        backoff_dist_cm=DEFAULT_BACKOFF_DISTANCE_CM,
        clear_dist_cm=DEFAULT_CLEAR_DISTANCE_CM,
        cruise_speed=DEFAULT_CRUISE_SPEED,
        reverse_speed=DEFAULT_REVERSE_SPEED,
        turn_speed=DEFAULT_TURN_SPEED,
        reverse_duration=DEFAULT_REVERSE_DURATION,
        turn_duration=DEFAULT_TURN_PULSE_DURATION,
        max_avoid_attempts=DEFAULT_MAX_AVOID_ATTEMPTS,
        is_simulation=False
    ):
        self.motor = motor_controller
        self.plan_provider = plan_provider
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin

        # Configuration parameters
        self.stop_dist_cm = float(stop_dist_cm)
        self.backoff_dist_cm = float(backoff_dist_cm)
        self.clear_dist_cm = float(clear_dist_cm)
        self.cruise_speed = float(cruise_speed)
        self.reverse_speed = float(reverse_speed)
        self.turn_speed = float(turn_speed)
        self.reverse_duration = float(reverse_duration)
        self.turn_duration = float(turn_duration)
        self.max_avoid_attempts = int(max_avoid_attempts)
        self.is_simulation = is_simulation

        # State tracking
        self.state = AutonomousState.IDLE
        self.is_active = False
        self._thread = None
        self._lock = threading.RLock()

        # Telemetry / Decision variables
        self.latest_distance_cm = 400.0
        self.avoidance_attempts = 0
        self.consecutive_sensor_fails = 0
        self.last_turn_direction = None
        self.turn_history = []
        self._recent_distances = deque(maxlen=3)

        self._gpio_initialized = False
        self._init_gpio()

    def _init_gpio(self):
        """Initializes RPi.GPIO pins for HC-SR04 if running on physical hardware."""
        if self.is_simulation:
            return
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.trig_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            GPIO.output(self.trig_pin, False)
            self._gpio_initialized = True
            logger.info(f"Ultrasonic pins initialized: TRIG={self.trig_pin}, ECHO={self.echo_pin}")
        except Exception as e:
            logger.warning(f"RPi.GPIO unavailable for ultrasonic driver ({e}). Running in simulated sensor mode.")
            self._gpio_initialized = False
            self.is_simulation = True

    def read_distance(self, timeout=0.055):
        """
        Reads one distance sample from the HC-SR04 ultrasonic sensor.
        Returns:
          - distance in cm (2.0 to 400.0) if an obstacle is within range.
          - 400.0 cm if path is completely clear / out-of-range (HC-SR04 internal timeout).
          - None if sensor is physically disconnected (ECHO pin never goes HIGH).
        """
        if self.is_simulation:
            return getattr(self, "_sim_distance", 50.0)

        try:
            import RPi.GPIO as GPIO
            # 30ms settling delay between pings
            GPIO.output(self.trig_pin, False)
            time.sleep(0.03)

            # Send 15µs trigger pulse
            GPIO.output(self.trig_pin, True)
            time.sleep(0.000015)
            GPIO.output(self.trig_pin, False)

            # Wait for echo to go HIGH
            deadline_start = time.time() + 0.04
            pulse_start = time.time()
            while GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start > deadline_start:
                    # Echo never went HIGH: hardware disconnected or no trigger response
                    return None

            # Wait for echo to return LOW
            deadline_end = time.time() + timeout
            pulse_end = time.time()
            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end > deadline_end:
                    # Echo took longer than timeout: completely clear path (> 400cm)
                    return 400.0

            duration = pulse_end - pulse_start
            distance_cm = duration * 17150.0

            # When open room has no bounce, HC-SR04 pulse length reaches max (~820cm)
            if distance_cm >= 400.0:
                return 400.0

            if distance_cm >= 2.0:
                return round(distance_cm, 1)

            return None
        except Exception as e:
            logger.debug(f"Sensor read exception: {e}")
            return None

    def get_filtered_distance(self):
        """
        Samples distance with a moving median filter across recent valid readings
        to eliminate acoustic noise, multipath reflections, and sporadic timeouts.
        """
        dist = self.read_distance()
        with self._lock:
            if dist is not None:
                self.consecutive_sensor_fails = 0
                self._recent_distances.append(dist)
                sorted_vals = sorted(self._recent_distances)
                median_dist = sorted_vals[len(sorted_vals) // 2]
                self.latest_distance_cm = median_dist
                return median_dist
            else:
                self.consecutive_sensor_fails += 1
                # If we have recent valid readings and only brief dropout, use last valid reading
                if self.consecutive_sensor_fails < DEFAULT_SENSOR_FAIL_LIMIT and self._recent_distances:
                    return self._recent_distances[-1]
                return None

    def start(self, force_restart=False):
        """Starts Autonomous Driving or resumes from blocked/obstacle state."""
        with self._lock:
            if self.is_active and not force_restart:
                # If currently blocked or halted by obstacle, resume immediately!
                if self.state in (AutonomousState.OBSTACLE_STOP, AutonomousState.BLOCKED_WAITING, AutonomousState.SENSOR_FAULT):
                    logger.info("Autonomous mode was halted/blocked; triggering resume.")
                    self.resume()
                return

            self.is_active = True
            self.avoidance_attempts = 0
            self.consecutive_sensor_fails = 0
            self.turn_history.clear()
            self.state = AutonomousState.CRUISING

            # Release any motor e-stops
            if hasattr(self.motor, "release_emergency_stop"):
                self.motor.release_emergency_stop()

            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(
                    target=self._autonomous_navigation_loop,
                    name="AutonomousNavigationLoop",
                    daemon=True
                )
                self._thread.start()
            logger.info(">>> LUMINA AUTONOMOUS MODE ACTIVATED <<<")

    def resume(self):
        """Explicit resume after obstacle stop, blocked waiting, or manual pause."""
        with self._lock:
            logger.info(">>> LUMINA AUTONOMOUS MODE RESUMED BY USER <<<")
            self.avoidance_attempts = 0
            self.consecutive_sensor_fails = 0
            self.turn_history.clear()
            self.state = AutonomousState.CRUISING

            if hasattr(self.motor, "release_emergency_stop"):
                self.motor.release_emergency_stop()

            if not self.is_active or self._thread is None or not self._thread.is_alive():
                self.is_active = True
                self._thread = threading.Thread(
                    target=self._autonomous_navigation_loop,
                    name="AutonomousNavigationLoop",
                    daemon=True
                )
                self._thread.start()

    def stop(self, state=AutonomousState.IDLE):
        """Gracefully and immediately stops autonomous driving and halts all motors."""
        with self._lock:
            self.is_active = False
            self.state = state
            if self.motor:
                try:
                    self.motor.stop()
                except Exception as e:
                    logger.error(f"Error stopping motors: {e}")
            logger.info(f"Autonomous Mode halted. State set to: {self.state}")

    def emergency_stop(self):
        """Emergency cut: stops autonomous mode and asserts motor controller emergency stop."""
        with self._lock:
            self.stop(state=AutonomousState.EMERGENCY_STOP)
            if hasattr(self.motor, "emergency_stop"):
                self.motor.emergency_stop()
            logger.warning("[SAFETY] Autonomous Emergency Stop Triggered!")

    def _choose_intelligent_turn_direction(self):
        """
        Intelligently decides turn direction (left or right):
        - Prevents always turning the same way (not a fixed sequence).
        - Prevents infinite ping-pong loops by alternating with backoff scaling.
        - Expands exploration radius on successive failed attempts.
        """
        attempt = self.avoidance_attempts

        if attempt == 0:
            # First attempt: randomize or choose alternating direction
            direction = "right" if random.random() > 0.5 else "left"
        elif attempt == 1:
            # Second attempt: try the opposite direction of attempt 0
            prev = self.turn_history[-1] if self.turn_history else "right"
            direction = "left" if prev == "right" else "right"
        elif attempt == 2:
            # Third attempt: turn in opposite direction with wider angle
            direction = "right" if self.last_turn_direction == "left" else "left"
        else:
            # Fourth attempt: turn opposite
            direction = "left" if random.random() > 0.5 else "right"

        self.last_turn_direction = direction
        self.turn_history.append(direction)
        return direction

    def _autonomous_navigation_loop(self):
        """
        Primary closed-loop autonomous navigation state machine:
        Runs at ~15-20 Hz, continuously evaluating obstacle clearance.
        """
        logger.info("Autonomous navigation loop entered.")

        while self.is_active:
            try:
                # 1. Sample filtered distance
                dist = self.get_filtered_distance()

                # Safety Check: Sensor Fault Detection (physically disconnected)
                if dist is None:
                    if self.consecutive_sensor_fails >= DEFAULT_SENSOR_FAIL_LIMIT:
                        logger.error(f"[SAFETY FAULT] Ultrasonic sensor disconnected! Halting car immediately.")
                        self.state = AutonomousState.SENSOR_FAULT
                        self.motor.stop()
                        time.sleep(0.5)
                        continue

                current_dist = dist if dist is not None else 400.0

                # -------------------------------------------------------------
                # SITUATION A: CLEAR PATH (Vision Routing + Forward Cruise)
                # -------------------------------------------------------------
                if self.state == AutonomousState.CRUISING:
                    if current_dist > self.stop_dist_cm:
                        # 1. Query AI vision model & path planner for road hazard routing
                        ai_plan = self.plan_provider() if callable(self.plan_provider) else {}
                        evasion_mode = ai_plan.get("evasion_mode", "CENTER_TRACKING")

                        if evasion_mode == "EMERGENCY_BRAKE":
                            logger.info("[Vision Hazard] Lane-spanning crater detected! Stopping to calculate alternative path.")
                            self.motor.stop()
                            self.state = AutonomousState.OBSTACLE_STOP
                            time.sleep(0.1)
                        elif evasion_mode in ("EVADE_LEFT", "EVADE_RIGHT"):
                            steer = float(ai_plan.get("steering", 0.0))
                            # Keep high PWM (minimum 88%) during evasion to prevent stalling under load
                            throttle_norm = max(0.88, (self.cruise_speed * 0.95) / 100.0)
                            logger.info(f"[Vision Evasion] Routing around pothole: {evasion_mode} (steer={steer:+.2f})")
                            self.motor.set_drive_command(steer, throttle_norm)
                            time.sleep(0.06)
                        else:
                            # Path is clear: drive straight ahead
                            self.motor.set_drive_command(0.0, self.cruise_speed / 100.0)
                            time.sleep(0.06)

                        # Reset avoidance attempts on sustained clear path
                        if current_dist > self.clear_dist_cm:
                            self.avoidance_attempts = 0
                    else:
                        # ---------------------------------------------------------
                        # SITUATION B: OBSTACLE DETECTED -> IMMEDIATE STOP
                        # ---------------------------------------------------------
                        logger.info(f"[Obstacle Detected] Clearance {current_dist:.1f} cm <= {self.stop_dist_cm:.1f} cm! Stopping.")
                        self.motor.stop()
                        self.state = AutonomousState.OBSTACLE_STOP
                        time.sleep(0.08)

                # -------------------------------------------------------------
                # SITUATION C: ANALYZE AND FIND ALTERNATIVE PATH
                # -------------------------------------------------------------
                elif self.state == AutonomousState.OBSTACLE_STOP:
                    # If vehicle is dangerously close to obstacle, reverse slightly first
                    if current_dist < self.backoff_dist_cm:
                        logger.info(f"[Proximity Close: {current_dist:.1f} cm] Reversing {self.reverse_duration}s for turn clearance...")
                        self.state = AutonomousState.BACKING_OFF
                        self.motor.backward(self.reverse_speed)
                        time.sleep(self.reverse_duration)
                        self.motor.stop()
                        time.sleep(0.1)

                    self.state = AutonomousState.DECIDING_TURN

                elif self.state == AutonomousState.DECIDING_TURN:
                    # Check if car is boxed in or trapped after repeated failed turns
                    if self.avoidance_attempts >= self.max_avoid_attempts:
                        # -----------------------------------------------------
                        # SITUATION D: PATH BLOCKED IN EVERY ATTEMPT
                        # -----------------------------------------------------
                        logger.warning(
                            f"[Path Blocked in Every Attempt] Reached {self.avoidance_attempts} failed turn attempts! "
                            f"Halting car to prevent infinite loop. Waiting for clear path or user intervention."
                        )
                        self.state = AutonomousState.BLOCKED_WAITING
                        self.motor.stop()
                        continue

                    # Intelligently pick direction
                    direction = self._choose_intelligent_turn_direction()
                    
                    # Duration scales slightly with attempt to probe wider angles if small turn failed
                    turn_time = self.turn_duration * (1.0 + (self.avoidance_attempts * 0.25))
                    logger.info(f"[Evasion Attempt #{self.avoidance_attempts + 1}] Turning {direction.upper()} for {turn_time:.2f}s...")

                    self.state = AutonomousState.TURNING
                    if direction == "left":
                        self.motor.left(self.turn_speed)
                    else:
                        self.motor.right(self.turn_speed)

                    time.sleep(turn_time)
                    self.motor.stop()
                    # Allow vehicle physics/chassis to settle before pinging
                    time.sleep(0.12)

                    # Verify new direction clearance with multiple fresh readings
                    post_turn_readings = []
                    for _ in range(3):
                        d = self.read_distance()
                        if d is not None:
                            post_turn_readings.append(d)
                        time.sleep(0.04)

                    new_clearance = (sum(post_turn_readings) / len(post_turn_readings)) if post_turn_readings else 0.0
                    self.latest_distance_cm = new_clearance
                    logger.info(f"[Turn Complete] Post-turn clearance: {new_clearance:.1f} cm (required >= {self.clear_dist_cm:.1f} cm)")

                    if new_clearance >= self.clear_dist_cm:
                        logger.info(f"[Path Clear!] Resuming forward cruise.")
                        self.avoidance_attempts = 0
                        self.turn_history.clear()
                        self.state = AutonomousState.CRUISING
                    else:
                        logger.info(f"[Still Blocked] Path not clear ({new_clearance:.1f} cm < {self.clear_dist_cm:.1f} cm). Incrementing attempt.")
                        self.avoidance_attempts += 1
                        self.state = AutonomousState.DECIDING_TURN

                # -------------------------------------------------------------
                # SITUATION D: BLOCKED WAITING (Trapped Condition)
                # -------------------------------------------------------------
                elif self.state == AutonomousState.BLOCKED_WAITING:
                    # Vehicle is safely stationary. Periodically probe distance to see if obstacle moved away
                    time.sleep(0.5)
                    dist = self.get_filtered_distance()
                    if dist is not None and dist >= self.clear_dist_cm:
                        logger.info(f"[Obstacle Cleared in Blocked State] Distance is now {dist:.1f} cm! Resuming autonomous driving.")
                        self.resume()

                # -------------------------------------------------------------
                # SENSOR FAULT HANDLING
                # -------------------------------------------------------------
                elif self.state == AutonomousState.SENSOR_FAULT:
                    time.sleep(0.5)
                    # Attempt recovery
                    test_read = self.read_distance()
                    if test_read is not None:
                        logger.info(f"[Sensor Recovered] Distance: {test_read:.1f} cm. Resuming cruise.")
                        self.consecutive_sensor_fails = 0
                        self.state = AutonomousState.CRUISING

                time.sleep(0.02)

            except Exception as e:
                logger.error(f"Unexpected error in autonomous navigation loop: {e}", exc_info=True)
                if self.motor:
                    try:
                        self.motor.stop()
                    except Exception:
                        pass
                time.sleep(0.2)

        # Loop exited: ensure motors are stopped
        if self.motor:
            try:
                self.motor.stop()
            except Exception:
                pass
        logger.info("Autonomous navigation loop terminated cleanly.")

    def get_telemetry(self):
        """Returns structured autonomous status dictionary for vehicle telemetry API."""
        with self._lock:
            return {
                "isActive": self.is_active,
                "state": self.state,
                "distanceCm": self.latest_distance_cm,
                "avoidanceAttempts": self.avoidance_attempts,
                "lastTurnDirection": self.last_turn_direction,
                "consecutiveFails": self.consecutive_sensor_fails,
                "thresholds": {
                    "stopDistanceCm": self.stop_dist_cm,
                    "backoffDistanceCm": self.backoff_dist_cm,
                    "clearDistanceCm": self.clear_dist_cm,
                    "cruiseSpeed": self.cruise_speed
                }
            }
