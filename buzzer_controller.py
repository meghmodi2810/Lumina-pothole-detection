"""
LUMINA Autonomous IoT Car - Active/Passive Buzzer Controller
Acoustic Warning System & Tactical Vehicle Horn
Wired via NPN Transistor Low-Side Driver:
  - Raspberry Pi 5V (Pin 2 / 4) -> Buzzer +ve
  - Buzzer -ve -> Transistor Collector (Left leg / Collector)
  - Transistor Emitter (Right leg) -> Ground (Pin 6 / 9 / 14 / 20 / 25 / 30 / 34 / 39)
  - Transistor Base (Middle leg) -> Resistor (1k - 4.7k)
  - Resistor other leg -> GPIO 2 (BCM 2, Physical Header Pin 3)

Non-blocking multi-threaded acoustic alerts:
  1. Pothole Detected Alert: Distinctive double-chirp (80ms ON, 50ms OFF, 80ms ON).
  2. Urgent Obstacle Alarm: Rapid warning bursts when path clearance <= stop distance.
  3. Manual Horn: Tactical driver honk (continuous while held or 250ms pulse).
  4. Startup Boot Chime: Ascending 3-tone ready arpeggio.
  5. Self-Test Pass Chime: Double confirmation chime synced with Green Status LED.
"""

import os
import time
import queue
import logging
import threading
import subprocess

logger = logging.getLogger("BuzzerController")
if not logger.handlers:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] [Buzzer] %(message)s")

DEFAULT_BUZZER_PIN = 2  # BCM 2 (Physical Pin 3 on RPi 40-Pin Header)

# Detect RPi.GPIO availability
GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False


class BuzzerController:
    """
    Thread-safe, non-blocking buzzer controller supporting real hardware GPIO
    actuation with multi-tier CLI fallbacks and simulated test-mode logging.
    """

    def __init__(self, pin=DEFAULT_BUZZER_PIN, is_simulation=None):
        self.pin = pin
        self.is_simulation = is_simulation if is_simulation is not None else (not GPIO_AVAILABLE)
        self.is_active = False
        self._lock = threading.RLock()
        self._action_queue = queue.Queue(maxsize=16)
        self._stop_event = threading.Event()
        self._worker_thread = None

        # Acoustic state tracking
        self.total_beeps = 0
        self.last_beep_time = 0.0
        self.last_pattern = "NONE"

        # Initialize physical pin safely
        self._init_hardware_pin()

        # Start non-blocking background audio worker
        self._start_worker()

    def _init_hardware_pin(self):
        """Initializes GPIO 2 as OUTPUT and immediately forces LOW to prevent floating noise."""
        if not self.is_simulation and GPIO_AVAILABLE:
            try:
                GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
                GPIO.output(self.pin, GPIO.LOW)
                logger.info(f"Initialized Buzzer on BCM GPIO {self.pin} (Pin 3) with state LOW via RPi.GPIO.")
                return
            except Exception as e:
                logger.warning(f"RPi.GPIO setup failed for buzzer on GPIO {self.pin}: {e}")

        # If gpiozero is available
        try:
            from gpiozero import OutputDevice
            self._gz_device = OutputDevice(self.pin, active_high=True, initial_value=False)
            logger.info(f"Initialized Buzzer on GPIO {self.pin} via gpiozero OutputDevice.")
            return
        except Exception:
            self._gz_device = None

        # Fallback to pinctrl CLI if available on Bookworm
        try:
            subprocess.run(["pinctrl", "set", str(self.pin), "op", "dl"], check=False, capture_output=True, timeout=0.5)
        except Exception:
            pass

        if self.is_simulation:
            logger.info(f"BuzzerController initialized in SIMULATION mode on virtual GPIO {self.pin}.")

    def _set_pin_state(self, high: bool):
        """Low-level actuation of buzzer transistor gate with cascade fallback."""
        with self._lock:
            self.is_active = high

        if self.is_simulation:
            # High-fidelity simulated actuation
            return

        # Tier 1: RPi.GPIO
        if GPIO_AVAILABLE:
            try:
                GPIO.output(self.pin, GPIO.HIGH if high else GPIO.LOW)
                return
            except Exception:
                pass

        # Tier 2: gpiozero
        if hasattr(self, "_gz_device") and self._gz_device is not None:
            try:
                if high:
                    self._gz_device.on()
                else:
                    self._gz_device.off()
                return
            except Exception:
                pass

        # Tier 3: pinctrl CLI (Raspberry Pi OS 12 Bookworm)
        try:
            val = "dh" if high else "dl"
            subprocess.run(["pinctrl", "set", str(self.pin), "op", val], check=False, capture_output=True, timeout=0.2)
            return
        except Exception:
            pass

        # Tier 4: raspi-gpio CLI (Bullseye)
        try:
            val = "1" if high else "0"
            subprocess.run(["raspi-gpio", "set", str(self.pin), "op", "dh" if high else "dl"], check=False, capture_output=True, timeout=0.2)
            return
        except Exception:
            pass

        # Tier 5: gpioset (libgpiod)
        try:
            val = "1" if high else "0"
            subprocess.run(["gpioset", "gpiochip0", f"{self.pin}={val}"], check=False, capture_output=True, timeout=0.2)
            return
        except Exception:
            pass

    def _start_worker(self):
        """Starts the dedicated background daemon thread for acoustic pattern sequencing."""
        self._stop_event.clear()
        self._worker_thread = threading.Thread(target=self._worker_loop, name="BuzzerAudioThread", daemon=True)
        self._worker_thread.start()

    def _worker_loop(self):
        """Processes acoustic alert sequences without blocking caller threads."""
        while not self._stop_event.is_set():
            try:
                task = self._action_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if task is None:
                break

            action_type, params = task
            try:
                if action_type == "PATTERN":
                    # Sequence of (on_time_sec, off_time_sec) pairs
                    pattern_name = params.get("name", "CUSTOM")
                    sequence = params.get("sequence", [])
                    self.last_pattern = pattern_name
                    logger.debug(f"Acoustic sequence started: {pattern_name}")
                    for on_sec, off_sec in sequence:
                        if self._stop_event.is_set():
                            break
                        if on_sec > 0:
                            self._set_pin_state(True)
                            self.total_beeps += 1
                            self.last_beep_time = time.time()
                            time.sleep(on_sec)
                        self._set_pin_state(False)
                        if off_sec > 0:
                            time.sleep(off_sec)
                elif action_type == "STATE":
                    high = bool(params.get("high", False))
                    self._set_pin_state(high)
                    if high:
                        self.total_beeps += 1
                        self.last_beep_time = time.time()
                elif action_type == "PULSE":
                    duration = float(params.get("duration", 0.25))
                    self._set_pin_state(True)
                    self.total_beeps += 1
                    self.last_beep_time = time.time()
                    time.sleep(duration)
                    self._set_pin_state(False)
            except Exception as e:
                logger.error(f"Error executing buzzer task {action_type}: {e}")
                self._set_pin_state(False)
            finally:
                self._action_queue.task_done()

    def _enqueue_pattern(self, name: str, sequence: list, clear_queue: bool = False):
        """Schedules an acoustic pattern sequence."""
        if clear_queue:
            # Drain pending non-critical tones for urgent alarms
            while not self._action_queue.empty():
                try:
                    self._action_queue.get_nowait()
                    self._action_queue.task_done()
                except (queue.Empty, ValueError):
                    break
        try:
            self._action_queue.put_nowait(("PATTERN", {"name": name, "sequence": sequence}))
        except queue.Full:
            logger.warning(f"Buzzer action queue full; dropping pattern {name}")

    # =========================================================================
    # Public Acoustic Profiles
    # =========================================================================

    def pothole_alert(self):
        """
        Distinctive Double-Chirp (80ms ON, 50ms OFF, 80ms ON).
        Acoustically signals driver that a road pothole defect has been detected and logged.
        """
        logger.info("[Acoustic] Triggering POTHOLE DETECTED double-chirp alert.")
        self._enqueue_pattern("POTHOLE_ALERT", [(0.08, 0.05), (0.08, 0.0)])

    def obstacle_alert(self):
        """
        Urgent Warning Alarm (3 fast 90ms bursts: 90ms ON, 50ms OFF x 3).
        High-priority urgent acoustic warning that vehicle is closing in on an obstacle.
        """
        logger.info("[Acoustic] Triggering URGENT OBSTACLE proximity alarm.")
        self._enqueue_pattern(
            "OBSTACLE_URGENT",
            [(0.09, 0.05), (0.09, 0.05), (0.12, 0.0)],
            clear_queue=True,
        )

    def horn_pulse(self, duration_sec: float = 0.25):
        """
        Tactical driver horn chirp/pulse. Default duration 250ms.
        """
        dur = max(0.05, min(3.0, float(duration_sec)))
        logger.info(f"[Acoustic] Sounding HORN pulse ({dur:.2f}s).")
        try:
            self._action_queue.put_nowait(("PULSE", {"duration": dur}))
        except queue.Full:
            pass

    def horn_on(self):
        """Holds horn active continuously (e.g. while driver holds button)."""
        logger.info("[Acoustic] HORN ENGAGED (continuous).")
        try:
            self._action_queue.put_nowait(("STATE", {"high": True}))
        except queue.Full:
            pass

    def horn_off(self):
        """Releases continuous horn."""
        logger.info("[Acoustic] HORN RELEASED.")
        try:
            self._action_queue.put_nowait(("STATE", {"high": False}))
        except queue.Full:
            pass

    def startup_chime(self):
        """
        Ascending 3-tone cheerful boot arpeggio.
        Signals system boot, perception models loaded, and vehicle ready for mission.
        """
        logger.info("[Acoustic] Playing STARTUP boot chime.")
        self._enqueue_pattern(
            "STARTUP_CHIME",
            [(0.06, 0.04), (0.07, 0.04), (0.12, 0.0)]
        )

    def selftest_pass_chime(self):
        """
        Double confirmation chime synchronized with Green Status LED glow.
        Signals 100% PASS on all hardware self-test diagnostic subsystems.
        """
        logger.info("[Acoustic] Playing SELF-TEST PASS confirmation chime.")
        self._enqueue_pattern(
            "SELFTEST_PASS",
            [(0.10, 0.07), (0.22, 0.0)]
        )

    def beep(self, duration: float = 0.15):
        """Standard generic diagnostic beep."""
        dur = max(0.02, min(5.0, float(duration)))
        self._enqueue_pattern("BEEP", [(dur, 0.0)])

    def get_status(self) -> dict:
        """Returns acoustic system telemetry."""
        with self._lock:
            active = self.is_active
        return {
            "subsystem": "BUZZER",
            "pin": self.pin,
            "isSimulation": self.is_simulation,
            "isActive": active,
            "totalBeeps": self.total_beeps,
            "lastBeepTime": self.last_beep_time,
            "lastPattern": self.last_pattern,
            "queuePending": self._action_queue.qsize(),
            "wiring": {
                "positive": "Raspberry Pi 5V",
                "negative": "Transistor Collector",
                "emitter": "Ground (GND)",
                "base": f"Resistor to GPIO {self.pin} (Pin 3)"
            }
        }

    def cleanup(self):
        """Safely stops worker thread and ensures pin is driven LOW to prevent hum/noise."""
        self._stop_event.set()
        try:
            self._action_queue.put_nowait(None)
        except Exception:
            pass
        self._set_pin_state(False)
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=0.8)
        logger.info("BuzzerController safely stopped and pin returned to LOW.")
