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
import atexit
import shutil

logger = logging.getLogger("BuzzerController")
if not logger.handlers:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] [Buzzer] %(message)s")

DEFAULT_BUZZER_PIN = int(os.environ.get("LUMINA_BUZZER_PIN", 2))  # BCM 2 (Physical Pin 3 on RPi 40-Pin Header)

# -----------------------------------------------------------------------------
# Multi-Tier Hardware Detection (RPi.GPIO, lgpio, pinctrl, raspi-gpio, gpioset)
# -----------------------------------------------------------------------------

# 1. Check for real Raspberry Pi hardware environment
IS_LINUX = os.name == "posix"
IS_PI_HARDWARE = (
    os.path.exists("/proc/device-tree/model")
    or os.path.exists("/dev/gpiomem")
    or any(os.path.exists(f"/dev/gpiochip{i}") for i in (0, 1, 2, 3, 4))
)

# 2. Check CLI tool availability
PINCTRL_BIN = shutil.which("pinctrl")
RASPI_GPIO_BIN = shutil.which("raspi-gpio")
GPIOSET_BIN = shutil.which("gpioset")
CLI_GPIO_AVAILABLE = bool(PINCTRL_BIN or RASPI_GPIO_BIN or GPIOSET_BIN)

# 3. Detect RPi.GPIO availability (legacy Bullseye/Buster)
GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False

# 4. Detect lgpio availability (official Raspberry Pi OS 12 Bookworm library)
LGPIO_AVAILABLE = False
_lgpio_chip = None
_lgpio_mod = None
try:
    import lgpio
    # On Pi 5 main GPIO is chip 4; on Pi 4 / 3 / Zero it is chip 0
    for chip_candidate in (0, 4, 1, 2, 3):
        try:
            h = lgpio.gpiochip_open(chip_candidate)
            _lgpio_chip = h
            _lgpio_mod = lgpio
            LGPIO_AVAILABLE = True
            break
        except Exception:
            continue
except Exception:
    LGPIO_AVAILABLE = False

ANY_HARDWARE_AVAILABLE = GPIO_AVAILABLE or LGPIO_AVAILABLE or CLI_GPIO_AVAILABLE or IS_PI_HARDWARE


def get_active_driver_name() -> str:
    """Returns human-readable name of primary active GPIO driver."""
    if LGPIO_AVAILABLE:
        return "lgpio (Bookworm native)"
    if GPIO_AVAILABLE:
        return "RPi.GPIO (sysfs/mem)"
    if PINCTRL_BIN:
        return f"pinctrl CLI ({PINCTRL_BIN})"
    if RASPI_GPIO_BIN:
        return f"raspi-gpio CLI ({RASPI_GPIO_BIN})"
    if GPIOSET_BIN:
        return f"gpioset CLI ({GPIOSET_BIN})"
    if IS_PI_HARDWARE:
        return "Generic Pi Linux (sysfs/fallback)"
    return "Simulation / Mock"


class BuzzerController:
    """
    Thread-safe, non-blocking buzzer controller supporting real hardware GPIO
    actuation with multi-tier CLI fallbacks and simulated test-mode logging.
    """

    def __init__(self, pin=DEFAULT_BUZZER_PIN, is_simulation=None):
        self.pin = pin
        # Dual-Pin Actuation: Simultaneously drives both GPIO 2 (Physical Pin 3)
        # and GPIO 21 (Physical Pin 40). Whichever pin the user plugs their wire into,
        # it works instantly with zero configuration!
        self.active_pins = [2, 21] if self.pin in (2, 21) else [self.pin]
        self.is_simulation = bool(is_simulation) if is_simulation is not None else (not ANY_HARDWARE_AVAILABLE)
        self.is_active = False
        self._lock = threading.RLock()
        self._action_queue = queue.Queue(maxsize=16)
        self._stop_event = threading.Event()
        self._worker_thread = None

        # Acoustic state tracking
        self.total_beeps = 0
        self.last_beep_time = 0.0
        self.last_pattern = "NONE"

        # Initialize physical pins safely
        self._init_hardware_pin()

        # Ensure safe shutdown clamp when Python process terminates
        try:
            atexit.register(self.cleanup)
        except Exception:
            pass

        # Start non-blocking background audio worker
        self._start_worker()

    def _init_hardware_pin(self):
        """
        Initializes buzzer GPIO pins as OUTPUT and immediately forces LOW (0V).
        Supports both GPIO 2 (Pin 3) and GPIO 21 (Pin 40).
        """
        if self.is_simulation:
            logger.info(f"BuzzerController initialized in SIMULATION mode on virtual GPIO {self.pin} (Active: {self.active_pins}).")
            return

        for p in self.active_pins:
            pin_num = 3 if p == 2 else (40 if p == 21 else p)
            initialized = False

            # Tier 1: lgpio (official Raspberry Pi OS 12 Bookworm library)
            if LGPIO_AVAILABLE and _lgpio_mod and _lgpio_chip is not None:
                try:
                    _lgpio_mod.gpio_claim_output(_lgpio_chip, p, 0)
                    _lgpio_mod.gpio_write(_lgpio_chip, p, 0)
                    initialized = True
                    logger.info(f"Initialized Buzzer on BCM GPIO {p} (Pin {pin_num}) with state LOW via lgpio.")
                except Exception as e:
                    logger.debug(f"lgpio init failed on GPIO {p}: {e}")

            # Tier 2: RPi.GPIO (Bullseye / Buster)
            if GPIO_AVAILABLE:
                try:
                    GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
                    GPIO.output(p, GPIO.LOW)
                    initialized = True
                    logger.info(f"Initialized Buzzer on BCM GPIO {p} (Pin {pin_num}) with state LOW via RPi.GPIO.")
                except Exception as e:
                    logger.debug(f"RPi.GPIO setup failed for buzzer on GPIO {p}: {e}")

            # Tier 3: pinctrl CLI (Raspberry Pi OS 12 Bookworm)
            if PINCTRL_BIN:
                try:
                    res = subprocess.run([PINCTRL_BIN, "set", str(p), "op", "dl"], check=False, capture_output=True, timeout=0.3)
                    if res.returncode == 0:
                        initialized = True
                        logger.info(f"Initialized Buzzer on BCM GPIO {p} via pinctrl (dl).")
                except Exception:
                    pass

            # Tier 4: raspi-gpio CLI (Bullseye)
            if not initialized and RASPI_GPIO_BIN:
                try:
                    res = subprocess.run([RASPI_GPIO_BIN, "set", str(p), "op", "dl"], check=False, capture_output=True, timeout=0.3)
                    if res.returncode == 0:
                        initialized = True
                except Exception:
                    pass

            # Tier 5: gpioset (libgpiod)
            if not initialized and GPIOSET_BIN:
                for chip_str in ("gpiochip0", "gpiochip4", "0", "4"):
                    try:
                        res = subprocess.run([GPIOSET_BIN, chip_str, f"{p}=0"], check=False, capture_output=True, timeout=0.3)
                        if res.returncode == 0:
                            initialized = True
                            break
                    except Exception:
                        pass

    def _set_pin_state(self, high: bool):
        """Low-level actuation of buzzer transistor gate with dual-pin support."""
        with self._lock:
            self.is_active = high

        if self.is_simulation:
            # High-fidelity simulated actuation
            return

        for p in self.active_pins:
            actuated = False

            # Tier 1: lgpio (Bookworm native Python API, sub-millisecond)
            if LGPIO_AVAILABLE and _lgpio_mod and _lgpio_chip is not None:
                try:
                    _lgpio_mod.gpio_write(_lgpio_chip, p, 1 if high else 0)
                    actuated = True
                except Exception:
                    pass

            # Tier 2: RPi.GPIO
            if not actuated and GPIO_AVAILABLE:
                try:
                    GPIO.output(p, GPIO.HIGH if high else GPIO.LOW)
                    actuated = True
                except Exception:
                    pass

            # Tier 3: pinctrl CLI (Raspberry Pi OS 12 Bookworm)
            if not actuated and PINCTRL_BIN:
                try:
                    val = "dh" if high else "dl"
                    res = subprocess.run([PINCTRL_BIN, "set", str(p), "op", val], check=False, capture_output=True, timeout=0.2)
                    if res.returncode == 0:
                        actuated = True
                except Exception:
                    pass

            # Tier 4: raspi-gpio CLI (Bullseye)
            if not actuated and RASPI_GPIO_BIN:
                try:
                    val = "dh" if high else "dl"
                    res = subprocess.run([RASPI_GPIO_BIN, "set", str(p), "op", val], check=False, capture_output=True, timeout=0.2)
                    if res.returncode == 0:
                        actuated = True
                except Exception:
                    pass

            # Tier 5: gpioset (libgpiod)
            if not actuated and GPIOSET_BIN:
                val = "1" if high else "0"
                for chip in ("gpiochip0", "gpiochip4", "0", "4"):
                    try:
                        res = subprocess.run([GPIOSET_BIN, chip, f"{p}={val}"], check=False, capture_output=True, timeout=0.2)
                        if res.returncode == 0:
                            actuated = True
                            break
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
            "activePins": self.active_pins,
            "driver": get_active_driver_name(),
            "isSimulation": self.is_simulation,
            "isActive": active,
            "totalBeeps": self.total_beeps,
            "lastBeepTime": self.last_beep_time,
            "lastPattern": self.last_pattern,
            "queuePending": self._action_queue.qsize(),
            "wiring": {
                "positive": "Raspberry Pi 5V (Pin 2 or 4)",
                "negative": "Transistor Collector (Left leg)",
                "emitter": "Ground (GND, Pin 6 / 9 / 14 / 20 / 25 / 30 / 34 / 39)",
                "base": f"Resistor to GPIO {self.pin} (Physical Pin {3 if self.pin == 2 else (40 if self.pin == 21 else self.pin)})"
            }
        }

    def cleanup(self):
        """
        Safely stops worker thread and ensures pins are driven LOW to prevent hum/noise.
        Locks both GPIO 2 and GPIO 21 to OUTPUT LOW (0V).
        """
        self._stop_event.set()
        try:
            self._action_queue.put_nowait(None)
        except Exception:
            pass
        self._set_pin_state(False)

        # Explicitly lock physical pins to OUTPUT LOW across all drivers
        for p in getattr(self, "active_pins", [self.pin]):
            if LGPIO_AVAILABLE and _lgpio_mod and _lgpio_chip is not None:
                try:
                    _lgpio_mod.gpio_write(_lgpio_chip, p, 0)
                except Exception:
                    pass

            if not self.is_simulation and GPIO_AVAILABLE:
                try:
                    GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)
                    GPIO.output(p, GPIO.LOW)
                except Exception:
                    pass

            if PINCTRL_BIN:
                try:
                    subprocess.run([PINCTRL_BIN, "set", str(p), "op", "dl"], check=False, capture_output=True, timeout=0.2)
                except Exception:
                    pass
            elif RASPI_GPIO_BIN:
                try:
                    subprocess.run([RASPI_GPIO_BIN, "set", str(p), "op", "dl"], check=False, capture_output=True, timeout=0.2)
                except Exception:
                    pass
            elif GPIOSET_BIN:
                for chip_str in ("gpiochip0", "gpiochip4", "0", "4"):
                    try:
                        subprocess.run([GPIOSET_BIN, chip_str, f"{p}=0"], check=False, capture_output=True, timeout=0.2)
                        break
                    except Exception:
                        pass

        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=0.8)
        logger.info(f"BuzzerController on GPIO {self.pin} safely stopped and pins {self.active_pins} locked to LOW (0V).")
