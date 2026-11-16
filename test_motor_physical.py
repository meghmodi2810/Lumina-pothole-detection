#!/usr/bin/env python3
"""
LUMINA Autonomous IoT Car - Physical Motor & 4WD Power Balance Diagnostic
========================================================================
Interactive and automated diagnostic script to isolate:
  1. Front vs. Rear motor power imbalance
  2. Series vs. Parallel L298N terminal wiring faults
  3. Mechanical gearbox stiction & overtightened mounting screws
  4. L298N Channel A vs. Channel B balance

Run this directly on your Raspberry Pi:
  python test_motor_physical.py
"""

import os
import sys
import time
import shutil

print("=" * 70)
print("  LUMINA 4WD MOTOR HARDWARE & POWER BALANCE DIAGNOSTIC")
print("=" * 70)

try:
    from motor_controller import MotorController
    motor = MotorController()
    print(f"\n[Driver Status]")
    print(f"  * Hardware Mode : {'REAL GPIO (RPi.GPIO)' if motor.is_hardware else 'SIMULATION'}")
    print(f"  * Left Bank Pins: IN1={motor.in1}, IN2={motor.in2}, ENA={motor.ena}")
    print(f"  * Right Bank Pins: IN3={motor.in3}, IN4={motor.in4}, ENB={motor.enb}")
    print(f"  * Left Trim     : {motor.trim_left:.2f}")
    print(f"  * Right Trim    : {motor.trim_right:.2f}")
except Exception as e:
    print(f"  ! Failed to load MotorController: {e}")
    sys.exit(1)

print("""
----------------------------------------------------------------------
IMPORTANT: ELEVATE CHASSIS OFF THE GROUND BEFORE PROCEEDING!
Place the car on a cup, box, or stand so all 4 wheels spin freely in the air.
----------------------------------------------------------------------
""")

input("Press ENTER when the car wheels are elevated in the air...")

# Step 1: Left Bank Independent Test
print("\n[Step 1/3] Spinning LEFT BANK ONLY for 2.5 seconds (100% PWM)...")
print("  >> WATCH CLOSELY: BOTH the Front-Left AND Rear-Left wheels should spin!")
try:
    # Left bank forward, Right bank stopped
    motor._apply_raw_outputs(1, 0, 0, 0, 100.0, 0.0)
    time.sleep(2.5)
    motor.stop()
except Exception as e:
    print(f"  ! Error during Left Bank test: {e}")

time.sleep(1.0)

# Step 2: Right Bank Independent Test
print("\n[Step 2/3] Spinning RIGHT BANK ONLY for 2.5 seconds (100% PWM)...")
print("  >> WATCH CLOSELY: BOTH the Front-Right AND Rear-Right wheels should spin!")
try:
    # Left bank stopped, Right bank forward
    motor._apply_raw_outputs(0, 0, 1, 0, 0.0, 100.0)
    time.sleep(2.5)
    motor.stop()
except Exception as e:
    print(f"  ! Error during Right Bank test: {e}")

time.sleep(1.0)

# Step 3: All 4 Wheels Forward Test
print("\n[Step 3/3] Spinning ALL 4 WHEELS FORWARD for 2.5 seconds (100% PWM)...")
print("  >> Compare rotational speeds of all 4 wheels:")
try:
    motor._apply_raw_outputs(1, 0, 1, 0, 100.0, 100.0)
    time.sleep(2.5)
    motor.stop()
except Exception as e:
    print(f"  ! Error during All Wheels test: {e}")

motor.cleanup()

print("\n" + "=" * 70)
print("  MOTOR DIAGNOSTIC RESULTS & HARDWARE FIX GUIDE")
print("=" * 70)
print("""
DIAGNOSTIC MATRIX:

1. Did only the FRONT wheels spin during Step 1, and only REAR wheels during Step 2?
   -> DIAGNOSIS: Channel-Swap Wiring Fault!
      You wired Front motors to Channel A (OUT1/OUT2) and Rear motors to Channel B (OUT3/OUT4).
   -> FIX: Rewire so OUT1/OUT2 controls BOTH Left motors (Front-Left + Rear-Left in parallel),
           and OUT3/OUT4 controls BOTH Right motors (Front-Right + Rear-Right in parallel).

2. During Step 1, did Front-Left spin fast while Rear-Left barely moved (~30%) or stalled?
   -> DIAGNOSIS: SERIES WIRING FAULT (Most Common) or MECHANICAL STICTION!
   -> CHECK A (Wiring - Series vs. Parallel):
      * If wired in Series, voltage divides (e.g. 4.5V on front, 1.5V on rear).
      * FIX: Wire in PARALLEL. Twist both Left (+) wires together into OUT1.
             Twist both Left (-) wires together into OUT2.
   -> CHECK B (Mechanical Gearbox Compression):
      * The two long M3 screws holding the rear motor to the chassis may be overtightened.
        This squeezes the yellow plastic casing and binds the internal gears.
      * FIX: Loosen the M3 mounting screws on the rear gearboxes by 1/2 to 1 full turn.
   -> CHECK C (Wheel Rubbing Chassis):
      * The rear rubber wheels may be pressed too deep onto the motor shaft,
        causing the inner hub to rub against the acrylic chassis plate.
      * FIX: Pull the rear wheels out slightly by 1-2mm to eliminate friction.

3. Are all 4 motors weak when running together?
   -> DIAGNOSIS: Battery Voltage Sag under 4WD Load.
   -> FIX: Power the L298N 12V terminal with 2x 18650 Li-ion cells (7.4V) or 3S LiPo (11.1V).
           Ensure a common Ground wire connects L298N GND to Raspberry Pi GND.
""")
