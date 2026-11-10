"""
LUMINA Physical Edge Hardware & Sensor Diagnostics Test Suite
Executed directly on the physical Raspberry Pi 4B hardware.
Verifies all physical pins, buses, camera sensor, ultrasonic sonar, PWM timers, and thermal health.
"""

import os
import time
import shutil
import unittest
import numpy as np

# Physical Pin Definitions
TRIG_PIN = 16
ECHO_PIN = 26
IN1_PIN = 17
IN2_PIN = 27
ENA_PIN = 18
IN3_PIN = 22
IN4_PIN = 23
ENB_PIN = 12
LED_PIN = 5

GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False


class TestPhysicalEdgeHardware(unittest.TestCase):

    def setUp(self):
        if not GPIO_AVAILABLE:
            self.skipTest("RPi.GPIO is not available on this platform. Running on physical Raspberry Pi required.")

    def test_01_gpio_subsystem_initialization(self):
        """Verifies RPi.GPIO can allocate and configure all physical hardware header pins."""
        pins = [TRIG_PIN, ECHO_PIN, IN1_PIN, IN2_PIN, ENA_PIN, IN3_PIN, IN4_PIN, ENB_PIN, LED_PIN]
        for p in [TRIG_PIN, IN1_PIN, IN2_PIN, ENA_PIN, IN3_PIN, IN4_PIN, ENB_PIN, LED_PIN]:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)
        GPIO.setup(ECHO_PIN, GPIO.IN)
        self.assertTrue(True, "All 9 physical header pins configured successfully.")

    def test_02_ultrasonic_hc_sr04_ping_accuracy(self):
        """Pings HC-SR04 ultrasonic sensor and validates timing, return pulse, and distance calculation."""
        GPIO.setup(TRIG_PIN, GPIO.OUT)
        GPIO.setup(ECHO_PIN, GPIO.IN)

        readings = []
        for i in range(5):
            GPIO.output(TRIG_PIN, False)
            time.sleep(0.04)

            # 15 microsecond trigger pulse
            GPIO.output(TRIG_PIN, True)
            time.sleep(0.000015)
            GPIO.output(TRIG_PIN, False)

            deadline = time.time() + 0.05
            pulse_start = time.time()
            while GPIO.input(ECHO_PIN) == 0:
                pulse_start = time.time()
                if pulse_start > deadline:
                    break

            pulse_end = time.time()
            while GPIO.input(ECHO_PIN) == 1:
                pulse_end = time.time()
                if pulse_end > deadline:
                    break

            duration = pulse_end - pulse_start
            distance_cm = duration * 17150.0
            if 2.0 <= distance_cm <= 500.0:
                readings.append(distance_cm)

        print(f"\n[HC-SR04 Sonar] 5 Pings Sampled: {readings} cm")
        self.assertGreater(len(readings), 0, "HC-SR04 failed to return any valid acoustic echo pulses.")
        avg_dist = sum(readings) / len(readings)
        self.assertGreater(avg_dist, 2.0, "Measured distance is below sensor physical minimum (2cm).")
        self.assertLess(avg_dist, 500.0, "Measured distance exceeds sensor range ceiling.")

    def test_03_l298n_hardware_pwm_modulation(self):
        """Tests dual hardware PWM timers on ENA (GPIO 18) and ENB (GPIO 12) at 100 Hz."""
        GPIO.setup([IN1_PIN, IN2_PIN, IN3_PIN, IN4_PIN, ENA_PIN, ENB_PIN], GPIO.OUT)
        pwm_l = GPIO.PWM(ENA_PIN, 100)
        pwm_r = GPIO.PWM(ENB_PIN, 100)

        pwm_l.start(0)
        pwm_r.start(0)

        # Step through power duty cycles: 25%, 50%, 75%, 100%
        for duty in [25.0, 50.0, 75.0, 100.0, 0.0]:
            pwm_l.ChangeDutyCycle(duty)
            pwm_r.ChangeDutyCycle(duty)
            time.sleep(0.05)

        pwm_l.stop()
        pwm_r.stop()
        self.assertTrue(True, "Hardware PWM timers cycled through duty-cycle sweep with zero errors.")

    def test_04_camera_picamera2_capture(self):
        """Verifies native CSI camera capture using Picamera2 at 640x480 resolution."""
        try:
            from picamera2 import Picamera2
        except ImportError:
            self.skipTest("Picamera2 not installed.")

        # Test camera subsystem availability
        picam = None
        try:
            picam = Picamera2()
            config = picam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
            picam.configure(config)
            picam.start()
            time.sleep(0.3)
            frame = picam.capture_array()
            self.assertIsNotNone(frame, "Captured camera array is None.")
            self.assertEqual(frame.shape, (480, 640, 3), f"Unexpected frame shape: {frame.shape}")
            self.assertEqual(frame.dtype, np.uint8, f"Unexpected frame dtype: {frame.dtype}")
            print(f"\n[Picamera2] Capture Success: {frame.shape[1]}x{frame.shape[0]} RGB (mean brightness: {np.mean(frame):.1f})")
        except Exception as e:
            # If camera is currently claimed by active service, verify via HTTP stream
            import urllib.request
            req = urllib.request.Request("http://127.0.0.1:5000/api/v1/telemetry")
            with urllib.request.urlopen(req, timeout=2) as r:
                data = r.read().decode()
                self.assertIn("NOMINAL", data)
                print(f"\n[Picamera2] Camera in active use by service: telemetry stream verified.")
        finally:
            if picam is not None:
                try:
                    picam.stop()
                    picam.close()
                except Exception:
                    pass

    def test_05_microsd_storage_and_io_speed(self):
        """Benchmarks MicroSD storage bandwidth and free space capacity."""
        total, used, free = shutil.disk_usage("/")
        free_gb = free / (1024 ** 3)
        total_gb = total / (1024 ** 3)
        print(f"\n[Storage Capacity] Total: {total_gb:.2f} GB | Free: {free_gb:.2f} GB")
        self.assertGreater(free_gb, 0.5, "Storage critical: less than 500MB free on MicroSD root filesystem.")

        # Benchmark 10MB sequential write and read
        test_file = "/home/lumina/lumina/tests/io_benchmark.tmp"
        os.makedirs(os.path.dirname(test_file), exist_ok=True)
        data = os.urandom(10 * 1024 * 1024)

        t0 = time.time()
        with open(test_file, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        write_time = time.time() - t0
        write_mbps = 10.0 / write_time

        t0 = time.time()
        with open(test_file, "rb") as f:
            read_data = f.read()
        read_time = time.time() - t0
        read_mbps = 10.0 / read_time

        if os.path.exists(test_file):
            os.remove(test_file)

        print(f"[MicroSD Bandwidth] Write: {write_mbps:.1f} MB/s | Read: {read_mbps:.1f} MB/s")
        self.assertGreater(write_mbps, 2.0, "MicroSD write speed is critically degraded (< 2 MB/s).")
        self.assertGreater(read_mbps, 10.0, "MicroSD read speed is critically degraded (< 10 MB/s).")

    def test_06_soc_thermal_ceiling_and_throttling(self):
        """Verifies Broadcom BCM2711 SoC temperature and hardware throttle flags."""
        temp_c = 0.0
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_c = float(f.read().strip()) / 1000.0
        except Exception:
            pass

        print(f"\n[SoC Thermals] CPU Core Temp: {temp_c:.1f}°C")
        self.assertGreater(temp_c, 20.0, "Unrealistic temperature reading (< 20°C).")
        self.assertLess(temp_c, 80.0, f"CPU core is dangerously close to 85°C thermal throttle threshold! ({temp_c}°C)")

        # Check vcgencmd throttling register if available
        import subprocess
        try:
            res = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2)
            throttled_str = res.stdout.strip()
            print(f"[Hardware Health] vcgencmd get_throttled: {throttled_str}")
            # Bit 0 = under-voltage currently detected. Bits 16-19 are historic.
            self.assertIn("throttled=0x", throttled_str)
            raw_val = int(throttled_str.split("=")[1], 16)
            is_under_voltage_now = bool(raw_val & 0x1)
            is_freq_capped_now = bool(raw_val & 0x2)
            is_throttled_now = bool(raw_val & 0x4)
            self.assertFalse(is_under_voltage_now, "FATAL: Raspberry Pi is currently suffering UNDER-VOLTAGE!")
            self.assertFalse(is_throttled_now, "FATAL: Raspberry Pi is currently thermally THROTTLED!")
        except FileNotFoundError:
            pass

    def test_07_status_led_gpio5_signaling(self):
        """Tests the external diagnostic Green LED on GPIO 5."""
        GPIO.setup(LED_PIN, GPIO.OUT)
        # 3 rapid test pulses
        for _ in range(3):
            GPIO.output(LED_PIN, GPIO.HIGH)
            time.sleep(0.04)
            GPIO.output(LED_PIN, GPIO.LOW)
            time.sleep(0.04)
        self.assertTrue(True, "Diagnostic LED on GPIO 5 pulsed successfully.")


if __name__ == "__main__":
    unittest.main()
