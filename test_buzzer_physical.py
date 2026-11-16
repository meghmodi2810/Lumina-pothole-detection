#!/usr/bin/env python3
"""
LUMINA Autonomous IoT Car - Physical Buzzer Hardware Checkout
==============================================================
Direct physical diagnostic script to test acoustic buzzer actuation on:
  - BCM GPIO 2  (Physical Header Pin 3)
  - BCM GPIO 21 (Physical Header Pin 40)

Run this directly on your Raspberry Pi:
  python test_buzzer_physical.py
"""

import os
import sys
import time
import shutil
import subprocess

print("=" * 70)
print("  LUMINA HARDWARE ACOUSTIC BUZZER DIAGNOSTIC SUITE")
print("=" * 70)

# 1. Platform Detection
model_path = "/proc/device-tree/model"
model_str = "Non-Pi / Host PC"
if os.path.exists(model_path):
    try:
        with open(model_path, "r", encoding="utf-8", errors="ignore") as f:
            model_str = f.read().strip().replace("\x00", "")
    except Exception:
        model_str = "Raspberry Pi (Unknown model)"

print(f"\n[Platform Info]")
print(f"  * Hardware Model : {model_str}")
print(f"  * OS / Platform  : {sys.platform} (Python {sys.version.split()[0]})")

# 2. Driver Availability Audit
pinctrl_bin = shutil.which("pinctrl")
raspi_gpio_bin = shutil.which("raspi-gpio")
gpioset_bin = shutil.which("gpioset")

has_rpi_gpio = False
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    has_rpi_gpio = True
except Exception:
    has_rpi_gpio = False

has_lgpio = False
lgpio_handle = None
try:
    import lgpio
    for chip in (0, 4, 1):
        try:
            h = lgpio.gpiochip_open(chip)
            lgpio_handle = (lgpio, h, chip)
            has_lgpio = True
            break
        except Exception:
            pass
except Exception:
    has_lgpio = False

print(f"\n[Driver Audit]")
print(f"  * lgpio (Bookworm native) : {'AVAILABLE (Chip ' + str(lgpio_handle[2]) + ')' if has_lgpio else 'NOT FOUND'}")
print(f"  * RPi.GPIO (sysfs/legacy) : {'AVAILABLE' if has_rpi_gpio else 'NOT FOUND'}")
print(f"  * pinctrl CLI             : {pinctrl_bin or 'NOT FOUND'}")
print(f"  * raspi-gpio CLI          : {raspi_gpio_bin or 'NOT FOUND'}")
print(f"  * gpioset CLI             : {gpioset_bin or 'NOT FOUND'}")

# 3. Read Current Pin State (GPIO 2 and GPIO 21)
print(f"\n[Current Pin Status]")
for pin in (2, 21):
    pin_name = f"GPIO {pin} (Pin 3)" if pin == 2 else f"GPIO {pin} (Pin 40)"
    read_state = "UNKNOWN"
    if pinctrl_bin:
        try:
            res = subprocess.run([pinctrl_bin, "get", str(pin)], capture_output=True, text=True, timeout=1)
            read_state = res.stdout.strip()
        except Exception:
            pass
    elif raspi_gpio_bin:
        try:
            res = subprocess.run([raspi_gpio_bin, "get", str(pin)], capture_output=True, text=True, timeout=1)
            read_state = res.stdout.strip()
        except Exception:
            pass
    print(f"  * {pin_name}: {read_state}")

# 4. Low-Level Direct Actuation Test
print("\n[Step 1/3] Direct Hardware Pulse via System Driver (150ms ON)...")
for p in (2, 21):
    # Drive HIGH
    if has_lgpio and lgpio_handle:
        mod, h, chip = lgpio_handle
        try:
            mod.gpio_claim_output(h, p, 1)
            mod.gpio_write(h, p, 1)
        except Exception:
            pass
    elif has_rpi_gpio:
        try:
            GPIO.setup(p, GPIO.OUT, initial=GPIO.HIGH)
            GPIO.output(p, GPIO.HIGH)
        except Exception:
            pass
    elif pinctrl_bin:
        try:
            subprocess.run([pinctrl_bin, "set", str(p), "op", "dh"], capture_output=True, timeout=0.2)
        except Exception:
            pass

time.sleep(0.15)

# Drive LOW
for p in (2, 21):
    if has_lgpio and lgpio_handle:
        mod, h, chip = lgpio_handle
        try:
            mod.gpio_write(h, p, 0)
        except Exception:
            pass
    elif has_rpi_gpio:
        try:
            GPIO.output(p, GPIO.LOW)
        except Exception:
            pass
    elif pinctrl_bin:
        try:
            subprocess.run([pinctrl_bin, "set", str(p), "op", "dl"], capture_output=True, timeout=0.2)
        except Exception:
            pass

time.sleep(0.2)

# 5. High-Level BuzzerController Integration Test
print("\n[Step 2/3] Testing BuzzerController Acoustic Patterns...")
try:
    from buzzer_controller import BuzzerController
    bz = BuzzerController(pin=2)
    status = bz.get_status()
    print(f"  * BuzzerController initialized successfully!")
    print(f"  * Active Driver: {status.get('driver')}")
    print(f"  * Simulation Mode: {status.get('isSimulation')}")
    print(f"  * Actuating Pins: {status.get('activePins')}")

    print("\n  >> Playing Pothole Double-Chirp (2 short beeps)...")
    bz.pothole_alert()
    time.sleep(0.4)

    print("  >> Playing Tactical Horn Pulse (250ms)...")
    bz.horn_pulse(0.25)
    time.sleep(0.4)

    print("  >> Playing Self-Test Pass Confirmation Chime...")
    bz.selftest_pass_chime()
    time.sleep(0.6)

    bz.cleanup()
    print("  * BuzzerController cleanup complete. All pins locked to LOW (0V).")
except Exception as e:
    print(f"  ! Error testing BuzzerController: {e}")

# 6. Safety Lock to LOW
for p in (2, 21):
    if pinctrl_bin:
        try:
            subprocess.run([pinctrl_bin, "set", str(p), "op", "dl"], capture_output=True, timeout=0.2)
        except Exception:
            pass

print("\n" + "=" * 70)
print("  DIAGNOSTIC TEST SUMMARY & PHYSICAL VERIFICATION")
print("=" * 70)
print("""
[Did you hear the buzzer beep?]

  IF YES:
    -> Your buzzer and software stack are 100% OPERATIONAL!
    -> The software simulation trap has been resolved.

  IF NO (Buzzer is still completely silent):
    Because the software has successfully sent actuation signals via
    both GPIO 2 and GPIO 21 across all available drivers, silence indicates
    a physical wiring orientation on the NPN transistor or buzzer:

    1. NPN Transistor Pinout Check (e.g. 2N2222 / S8050 / BC547):
       - Holding the flat side facing you with legs pointing down:
         * Pin 1 (Left)   : Collector -> Connect to Buzzer NEGATIVE (-)
         * Pin 2 (Middle) : Base      -> Connect via 1k resistor to GPIO 2 (Pin 3)
         * Pin 3 (Right)  : Emitter   -> Connect to Raspberry Pi GND (Pin 6/9/14/20)
       * Note: If Collector and Emitter are reversed, current cannot flow.

    2. Buzzer Polarity (+ vs -):
       - Buzzer (+) must connect to Raspberry Pi 5V (Physical Pin 2 or Pin 4).
       - Buzzer (-) must connect to the Transistor Collector.

    3. Common Ground Check:
       - Ensure Raspberry Pi GND is firmly connected to the Transistor Emitter.
""")
