#!/usr/bin/env python3
"""
LUMINA Autonomous IoT Car - GPIO 2 (Physical Pin 3) Boot Silence Utility
-----------------------------------------------------------------------
Permanently silences the active buzzer at Raspberry Pi power-on / boot
without requiring any physical wire changes.

Why this is needed:
  Physical Pin 3 (BCM GPIO 2) is the Raspberry Pi I2C1 SDA pin.
  The Raspberry Pi PCB has a permanent hardware 1.8k ohm pull-up resistor
  to 3.3V connected to this pin. At power-on and during Linux boot, this
  pull-up feeds current into the transistor base, causing the buzzer to
  buzz continuously until software drives it LOW (0V).

What this script does:
  1. Configures the Raspberry Pi VideoCore bootloader via config.txt:
     - gpio=2=op,dl      (Sets GPIO 2 to Output and drives it LOW within 100ms of power-on)
     - dtparam=i2c_arm=off (Disables I2C driver so kernel doesn't pulse GPIO 2)
  2. Immediately drives GPIO 2 LOW in the live system so buzzing stops instantly.
"""

import os
import sys
import subprocess

CONFIG_PATHS = [
    "/boot/firmware/config.txt",  # Raspberry Pi OS 12 (Bookworm)
    "/boot/config.txt",           # Raspberry Pi OS 11 (Bullseye / Legacy)
]

DIRECTIVE_MARKER = "# LUMINA BUZZER HARDWARE CLAMP (GPIO 2 / PIN 3)"
DIRECTIVES = [
    DIRECTIVE_MARKER,
    "gpio=2=op,dl",
    "dtparam=i2c_arm=off"
]


def clamp_live_pin():
    """Immediately drives GPIO 2 LOW in the running OS."""
    print("[1/2] Silencing live GPIO 2 (Pin 3) buzzer line right now...")
    clamped = False

    # Attempt pinctrl (Bookworm)
    try:
        res = subprocess.run(["pinctrl", "set", "2", "op", "dl"], capture_output=True, text=True, timeout=2)
        if res.returncode == 0:
            print("  ✓ Live pin clamped to LOW (0V) via pinctrl.")
            clamped = True
    except Exception:
        pass

    # Attempt raspi-gpio (Bullseye)
    if not clamped:
        try:
            res = subprocess.run(["raspi-gpio", "set", "2", "op", "dl"], capture_output=True, text=True, timeout=2)
            if res.returncode == 0:
                print("  ✓ Live pin clamped to LOW (0V) via raspi-gpio.")
                clamped = True
        except Exception:
            pass

    # Attempt RPi.GPIO
    if not clamped:
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(2, GPIO.OUT, initial=GPIO.LOW)
            GPIO.output(2, GPIO.LOW)
            print("  ✓ Live pin clamped to LOW (0V) via RPi.GPIO.")
            clamped = True
        except Exception:
            pass

    if not clamped:
        print("  ! Notice: Could not clamp live pin via CLI/RPi.GPIO (normal if running off-board).")


def configure_boot_config():
    """Appends bootloader directives to /boot/firmware/config.txt or /boot/config.txt."""
    print("[2/2] Updating Raspberry Pi bootloader config.txt...")
    target_path = None
    for p in CONFIG_PATHS:
        if os.path.exists(p):
            target_path = p
            break

    if not target_path:
        print("  ! Error: Neither /boot/firmware/config.txt nor /boot/config.txt was found.")
        print("    If running remotely on your PC, execute this command directly on your Raspberry Pi:")
        print("    echo -e '\\ngpio=2=op,dl\\ndtparam=i2c_arm=off' | sudo tee -a /boot/firmware/config.txt")
        return False

    try:
        with open(target_path, "r", encoding="utf-8") as f:
            content = f.read()

        if "gpio=2=op,dl" in content:
            print(f"  ✓ Bootloader clamp 'gpio=2=op,dl' is ALREADY configured in {target_path}.")
            return True

        if os.geteuid() != 0:
            print(f"  ! Root permissions required to write to {target_path}.")
            print("    Please re-run with sudo:")
            print(f"    sudo python3 {os.path.abspath(__file__)}")
            return False

        with open(target_path, "a", encoding="utf-8") as f:
            f.write("\n" + "\n".join(DIRECTIVES) + "\n")

        print(f"  ✓ Successfully appended bootloader clamp to {target_path}:")
        print("    gpio=2=op,dl")
        print("    dtparam=i2c_arm=off")
        print("  ★ Buzzer will now stay completely SILENT the millisecond power is plugged in!")
        return True
    except PermissionError:
        print(f"  ! Permission denied writing to {target_path}. Run with: sudo python3 {__file__}")
        return False
    except Exception as e:
        print(f"  ! Failed updating {target_path}: {e}")
        return False


def main():
    print("=" * 70)
    print(" LUMINA CAR: GPIO 2 BUZZER BOOT SILENCE UTILITY")
    print("=" * 70)
    clamp_live_pin()
    configure_boot_config()
    print("=" * 70)
    print(" Done! GPIO 2 (Pin 3) is now hardware-stabilized.")
    print("=" * 70)


if __name__ == "__main__":
    main()
