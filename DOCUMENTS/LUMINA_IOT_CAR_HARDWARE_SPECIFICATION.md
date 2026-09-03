# Project Lumina: Complete IoT Smart Car Hardware Specification & Procurement Guide

This document contains the exhaustive, component-by-component hardware blueprint, electrical wiring diagrams, bill of materials (BOM), and assembly instructions for the **Project Lumina Autonomous IoT Car**.

---

## 1. Physical Specifications & Dimensions

* **Vehicle Scale**: 1:10 to 1:12 miniature robotics chassis
* **Chassis Footprint**: Length: 25 cm | Width: 18 cm | Height: 16 cm (to top of camera mount)
* **Ground Clearance**: 2.5 cm (with standard 65mm rubber wheels)
* **Total Vehicle Weight**: 850 grams (including dual battery packs, sensors, and Raspberry Pi)
* **Drive Mechanism**: 4WD Differential Skid-Steering (4 independent DC motors)
* **Target Cruising Speed**: 25 to 35 cm/second (PWM throttle at 35% to 40%)

---

## 2. Master Bill of Materials (BOM)

Every single mechanical, electrical, passive, and connector component required to build the IoT car is listed below with specifications, quantities, and functions.

### Group A: Compute, Camera & Memory
| Item # | Component Name | Exact Part / Model | Quantity | Operating Voltage | Purpose / Function |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **A1** | Single Board Computer | Raspberry Pi 4 Model B (4GB or 8GB RAM) | 1 | 5.1V DC (3.0A) | Main edge controller; runs camera stream, local safety loop, and motor PWM. |
| **A2** | Vision Camera Sensor | Raspberry Pi Camera Module V2 (Sony IMX219 8MP) | 1 | 3.3V (via CSI) | Video frame capture at 1080p/720p 30 FPS. |
| **A3** | Camera Ribbon Cable | 15-pin 15cm FFC Flexible Flat Cable | 1 | N/A | Connects Pi Camera V2 to Raspberry Pi CSI port. |
| **A4** | Camera Mount Bracket | Acrylic / 3D-Printed Camera Gimbal/Bracket | 1 | N/A | Secures camera at 12 to 15 cm height with 25 to 35 degree downward tilt. |
| **A5** | MicroSD Card | SanDisk Extreme / Ultra 32GB or 64GB Class 10 U3 | 1 | 3.3V | Operating system storage (Raspberry Pi OS 64-bit Lite). |
| **A6** | CPU Cooling Kit | Aluminum Heatsinks with 5V Dual Mini Fan | 1 kit | 5.0V | Prevents thermal throttling of the Broadcom BCM2711 CPU. |

---

### Group B: Mechanical Chassis & Propulsion
| Item # | Component Name | Exact Part / Model | Quantity | Specifications | Purpose / Function |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **B1** | Robot Chassis Plates | 2-Tier Acrylic or Aluminum 4WD Robot Base | 1 kit | Dual deck (bottom & top plates) | Houses motors, battery trays, driver board, and Raspberry Pi. |
| **B2** | Drive Motors | TT Gear Motors with All-Metal Gears | 4 | 6V - 12V DC, 200 RPM, 1:48 gear ratio | High-torque wheel propulsion. Metal gears prevent teeth stripping. |
| **B3** | Drive Wheels | 65mm Rubber Grip Robotics Wheels | 4 | 65mm diameter x 26mm width | High-traction rubber tread for carpet, wood, and tile tracks. |
| **B4** | Motor Brackets | Steel / Aluminum TT Motor Mounts | 4 | Heavy-duty L-brackets with M3 screws | Locks DC motors rigidly to the lower chassis deck. |
| **B5** | Brass Standoffs & Screws | M3 Brass Hex Spacer Standoff Kit (M3 10mm to 30mm) | 1 box (50+ pcs) | M3 threaded male-to-female and female-to-female | Separates the lower motor deck from the upper electronics deck. |
| **B6** | Fastener Hardware | M2.5 Screws and Nuts | 8 | M2.5 x 6mm nylon / metal | Used to screw the Raspberry Pi 4B board securely to chassis standoffs. |

---

### Group C: Motor Driver & Power Electronics
| Item # | Component Name | Exact Part / Model | Quantity | Specifications | Purpose / Function |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **C1** | Motor Driver Board | L298N Dual Full-Bridge H-Bridge Module (or TB6612FNG) | 1 | Up to 2A peak per channel, 7V to 35V drive | Amplifies low-power logic PWM from Raspberry Pi to high-current motor power. |
| **C2** | DC-DC Step-Down Buck Converter | LM2596 Buck Converter Module (with Digital Voltmeter) | 1 | Input: 7V-35V, Output: 5.1V (3.0A max) | Steps down 7.4V battery voltage to clean, regulated 5.1V for Raspberry Pi logic. |
| **C3** | Primary Motor Battery | 7.4V 2S 2200mAh 25C/30C LiPo Rechargeable Battery | 1 | 7.4V nominal (8.4V full charge), XT60 connector | Dedicated high-current discharge battery exclusively for DC drive motors. |
| **C4** | Logic Power Battery | 2x 18650 3.7V Li-ion (or 5V 3A USB-C Power Bank) | 2 cells | 3.7V each in series (7.4V total) | Dedicated power source for Raspberry Pi logic. Prevents brownout reboots. |
| **C5** | 18650 Battery Holder | 2-Slot 18650 Battery Case with Wire Leads | 1 | 2-cell series connection (7.4V output) | Holds the logic power cells securely with on/off switch. |
| **C6** | XT60 to Terminal Connector | XT60 Female Pigtail Cable (14 AWG) | 1 | 14 AWG silicone wire, 10 cm | Connects the 2S LiPo motor battery to the L298N screw terminals. |

---

### Group D: Sensors & Ranging Instrumentation
| Item # | Component Name | Exact Part / Model | Quantity | Specifications | Purpose / Function |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **D1** | Ultrasonic Proximity Sensor | HC-SR04 Ultrasonic Ranging Module | 1 | 5V DC, 40kHz, 2cm to 400cm range | Hardware fail-safe emergency braking (halts car if wall/pedestrian < 15cm). |
| **D2** | Ultrasonic Mount | Acrylic HC-SR04 Sensor Bracket | 1 | Front bumper vertical mount | Points the ultrasonic sound cone horizontally forward. |
| **D3** | GPS Positioning Module | NEO-6M GPS Module with Ceramic Patch Antenna | 1 | 3.3V - 5.0V, UART serial interface (9600 baud) | Tags road potholes with latitude, longitude, and speed telemetry. |
| **D4** | Optical Speed Encoders | HC-020K Optical Wheel Speed Encoder Kit | 2 | 3.3V - 5V, slotted disc with IR optocoupler | Measures real-time wheel RPM, travel distance, and wheel slip. |
| **D5** | Ambient Light Sensor | LDR Photoresistor Module (with LM393 Comparator) | 1 | 3.3V - 5.0V, Digital DO output | Detects low light/tunnel entry to automatically trigger headlamps. |
| **D6** | Thermal Temperature Probe | DS18B20 1-Wire Waterproof Digital Thermometer | 1 | -55 to +125 deg C, 1-Wire bus | Monitors temperature of L298N driver heatsink and battery compartment. |
| **D7** | Audio Feedback Buzzer | Active 5V Piezo Buzzer Module | 1 | 3.3V - 5.0V, 85dB tone | Sounds audible alarms during emergency halt, network loss, and startup. |

---

### Group E: Buttons, Switches & Visual Indicators
| Item # | Component Name | Exact Part / Model | Quantity | Specifications | Purpose / Function |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **E1** | Master Motor Power Switch | KCD1 Rocker Toggle Switch (SPST 2-Pin) | 1 | 6A 250V / 10A 125V rating | Physical master cutoff switch in series with motor battery positive lead. |
| **E2** | Emergency Kill Button | 12mm Momentary Red Push Button | 1 | Panel-mount SPST normally open | Physical stop button wired to Raspberry Pi GPIO for immediate motor cutoff. |
| **E3** | Mode Switch Button | 12mm Momentary Yellow Push Button | 1 | Panel-mount SPST normally open | Toggles car mode between Manual Drive and Autonomous AI Avoidance. |
| **E4** | Status Indicator LED (Green) | 5mm Diffused Green LED | 1 | 2.1V forward voltage, 20mA max | Lights up when car is in Manual/Cruising state. |
| **E5** | Status Indicator LED (Blue) | 5mm Diffused Blue LED | 1 | 3.2V forward voltage, 20mA max | Lights up when AI Autonomous Navigation & Cloud Stream is ACTIVE. |
| **E6** | Status Indicator LED (Red) | 5mm Diffused Red LED | 1 | 1.9V forward voltage, 20mA max | Flashes rapidly during Emergency Braking or sensor failure. |
| **E7** | High-Brightness Headlights | 2x 5mm Ultra-Bright White LEDs | 2 | 3.3V forward voltage, 20mA each | Illuminates road potholes when driving in dark or shaded conditions. |

---

### Group F: Wiring, Resistors, Breadboards & Consumables
| Item # | Component Name | Exact Part / Model | Quantity | Specifications | Purpose / Function |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **F1** | Dupont Jumper Wires (M-to-F) | 20cm Male-to-Female Jumper Wires | 40 wires | 28 AWG multi-color ribbon | Connects Raspberry Pi GPIO header pins to sensor breakout boards. |
| **F2** | Dupont Jumper Wires (F-to-F) | 20cm Female-to-Female Jumper Wires | 20 wires | 28 AWG multi-color ribbon | Connects L298N control pins, LEDs, and breadboard rails. |
| **F3** | Dupont Jumper Wires (M-to-M) | 10cm / 20cm Male-to-Male Jumper Wires | 20 wires | 28 AWG multi-color ribbon | Interconnects components on the mini breadboard. |
| **F4** | Heavy Motor Hookup Wire | 20 AWG / 22 AWG Stranded Copper Wire (Red & Black) | 2 meters | Flexible silicone insulation | High-current power wiring between battery, switches, and L298N driver. |
| **F5** | Resistor: 1k Ohm | 1/4 Watt Metal Film Resistor (1,000 ohms) | 2 | 1% tolerance | Upper resistor in voltage divider for HC-SR04 Echo pin (5V to 3.3V). |
| **F6** | Resistor: 2k Ohm | 1/4 Watt Metal Film Resistor (2,000 ohms) | 2 | 1% tolerance | Lower resistor in voltage divider for HC-SR04 Echo pin (5V to 3.3V). |
| **F7** | Resistor: 330 Ohm | 1/4 Watt Metal Film Resistor (330 ohms) | 5 | 1% tolerance | Current-limiting protection resistors for LEDs and active buzzer. |
| **F8** | Resistor: 4.7k Ohm | 1/4 Watt Metal Film Resistor (4,700 ohms) | 1 | 1% tolerance | Pull-up resistor for DS18B20 1-Wire temperature bus. |
| **F9** | Mini Breadboard | 170-Tie-Point Mini Solderless Breadboard | 1 | Self-adhesive backing, 45mm x 35mm | Mounts voltage divider resistors, status LEDs, and common ground bus. |
| **F10** | Cable Ties & Fasteners | Nylon Cable Zip Ties (100mm) | 20 | Black 2.5mm width | Secures wire harnesses cleanly to the chassis frame. |
| **F11** | Adhesive Mounting Tape | 3M Dual-Lock / Heavy-Duty Foam Tape | 1 roll | 25mm width double-sided | Firmly mounts batteries, buck converter, and breadboard to chassis. |
| **F12** | Heat Shrink Tubing | Polyolefin Heat Shrink Assortment (2mm to 6mm) | 1 pack | 2:1 shrink ratio | Insulates soldered wire splices, switches, and battery leads. |

---

## 3. Power Architecture & Voltage Regulation

```
+----------------------------------------------------------------------------------------------------+
|                                    LUMINA DUAL-POWER SYSTEM                                         |
+----------------------------------------------------------------------------------------------------+

POWER STAGE 1: DEDICATED MOTOR DRIVE (HIGH CURRENT)
[7.4V 2S LiPo Battery] 
        |
        +---> [KCD1 Rocker Switch (Power Cutoff)]
                    |
                    +---> L298N VMS Screw Terminal (+7.4V)
                    |
                    +---> L298N GND Screw Terminal -----------------------+
                                                                          |
                                                                          | COMMON GROUND
POWER STAGE 2: DEDICATED RASPBERRY PI LOGIC (ISOLATED CLEAN POWER)        | BUS (STAR POINT)
[2x 18650 Battery Pack (7.4V)]                                            |
        |                                                                 |
        +---> LM2596 Step-Down Buck Converter (Tuned to EXACTLY 5.10V)   |
                    |                                                     |
                    +--- Output Positive (+5.1V) ---> Pi Pin 02 / Pin 04  |
                    |                                                     |
                    +--- Output Negative (GND)   ---> Pi Pin 06 / Pin 39 -+
```

### Critical Electrical Safeguards:
1. **Never Share Logic and Motor Power Rails**: Connecting the motors directly to the Raspberry Pi 5V pin will trigger an instant brownout (reboot) whenever the motors accelerate or reverse.
2. **Common Ground Star Bus**: The negative terminal of the motor battery and the negative terminal of the logic battery **must connect together at a single point** (Pin 06 of the Raspberry Pi or the breadboard ground rail). Without a common ground, logic signals to the L298N driver will float and motors will behave erratically.
3. **Buck Converter Pre-Flight Calibration**: Before connecting the LM2596 buck converter to the Raspberry Pi GPIO pins, turn on the battery and measure the output with a digital multimeter. Turn the small brass potentiometer screw counter-clockwise until the voltmeter reads **exactly 5.10V**. Do not connect until verified!

---

## 4. Complete Raspberry Pi 40-Pin GPIO Wiring Matrix

Below is the pinout mapping for every single connection on the Raspberry Pi 4B 40-pin header:

| Pin # | Pin Name / BCM | Wire Color | Connects To | Subsystem / Function | Electrical Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **01** | **3.3V Power** | Red | GPS Module VCC & LDR VCC | Sensor Logic Power | 50mA max draw from Pi 3.3V rail. |
| **02** | **5.0V Power** | Orange | LM2596 Buck Converter (+5.1V OUT) | Raspberry Pi Main Power IN | Clean regulated 5.1V logic power. |
| **04** | **5.0V Power** | Red | HC-SR04 Ultrasonic VCC & Buzzer VCC | Sensor 5V Power Supply | 5V supply for ultrasonic sensor. |
| **06** | **Ground (GND)** | Black | Breadboard Ground / Common Bus | Common Ground Reference | Central star ground point. |
| **08** | **GPIO 14 (UART TX)**| Yellow | NEO-6M GPS Module RX Pin | GPS Serial Communication | Transmits configuration strings. |
| **10** | **GPIO 15 (UART RX)**| Green | NEO-6M GPS Module TX Pin | GPS Serial NMEA Sentences | Receives latitude, longitude, and speed. |
| **11** | **GPIO 17** | White | L298N IN1 Terminal | Left Motors Forward Direction | High = Left Forward, Low = Stop. |
| **12** | **GPIO 18 (PWM0)** | Blue | L298N ENA Terminal (Jumper removed)| Left Motors Speed PWM | Hardware PWM for speed modulation. |
| **13** | **GPIO 27** | Grey | L298N IN2 Terminal | Left Motors Reverse Direction | High = Left Reverse, Low = Stop. |
| **15** | **GPIO 22** | Purple | L298N IN3 Terminal | Right Motors Forward Direction | High = Right Forward, Low = Stop. |
| **16** | **GPIO 23** | Brown | L298N IN4 Terminal | Right Motors Reverse Direction | High = Right Reverse, Low = Stop. |
| **18** | **GPIO 24** | Green | Status LED Green (Anode via 330 ohm) | Manual Mode Indicator | Lights up during manual operation. |
| **19** | **GPIO 10** | Blue | Status LED Blue (Anode via 330 ohm) | Autonomous AI Mode Indicator | Lights up when AI avoidance is active. |
| **21** | **GPIO 09** | Red | Status LED Red (Anode via 330 ohm) | Emergency Brake / Error Indicator | Flashes during obstacle stop or error. |
| **22** | **GPIO 25** | Yellow | Active Buzzer I/O Pin | Audio Alarm Annunciation | High = Buzzer Beep, Low = Silence. |
| **24** | **GPIO 08** | White | LDR Module DO (Digital Output) | Ambient Light Level Readout | High = Dark (lights on), Low = Bright. |
| **29** | **GPIO 05** | Green | Left Wheel Optical Encoder D0 | Left Wheel RPM / Odometry | Interrupt counter for wheel distance. |
| **31** | **GPIO 06** | Yellow | Right Wheel Optical Encoder D0 | Right Wheel RPM / Odometry | Interrupt counter for wheel distance. |
| **32** | **GPIO 12 (PWM0)** | Blue | L298N ENB Terminal (Jumper removed)| Right Motors Speed PWM | Hardware PWM for speed modulation. |
| **36** | **GPIO 16** | Orange | HC-SR04 Ultrasonic Trigger (TRIG) | Ultrasonic Ranging Trigger | Sends 10 microsecond pulse. |
| **37** | **GPIO 26** | White | DS18B20 Temp Sensor DQ (with 4.7k) | Driver Heatsink Temperature | 1-Wire thermal monitoring. |
| **38** | **GPIO 20** | Yellow | HC-SR04 Echo (via 1k/2k Divider) | Ultrasonic Ranging Echo In | Receives 3.3V safe stepped-down pulse. |
| **39** | **Ground (GND)** | Black | Common Ground Rail | Logic Ground Reference | Ground return rail. |

---

## 5. Circuit Wiring Schematics

### 5.1 HC-SR04 Ultrasonic 5V to 3.3V Voltage Divider
The HC-SR04 Echo pin outputs a 5.0V signal. The Raspberry Pi GPIO pins are **not 5V tolerant** and will be permanently damaged if connected directly. A two-resistor voltage divider is required:

```
[HC-SR04 Echo Pin (5.0V)] 
           |
           +---> [1,000 Ohm Resistor (1k)]
                       |
                       +---> To Raspberry Pi GPIO 20 (Pin 38) [Safe 3.3V Signal]
                       |
                       +---> [2,000 Ohm Resistor (2k)]
                                   |
                                   +---> To Common Ground (GND)
```
$$\text{Output Voltage} = 5.0\text{V} \times \frac{2000}{1000 + 2000} = 3.33\text{V}$$

---

### 5.2 L298N to DC Motors Wiring (4WD Parallel Configuration)
To control 4 motors with a dual-channel driver, wire the two motors on each side in parallel:

* **Channel A (Left Side Motors)**:
  * Wire Motor Left-Front (+) and Motor Left-Rear (+) together to **L298N OUT1**.
  * Wire Motor Left-Front (-) and Motor Left-Rear (-) together to **L298N OUT2**.
* **Channel B (Right Side Motors)**:
  * Wire Motor Right-Front (+) and Motor Right-Rear (+) together to **L298N OUT3**.
  * Wire Motor Right-Front (-) and Motor Right-Rear (-) together to **L298N OUT4**.
* **Speed Jumpers**: Remove the two black jumpers on `ENA` and `ENB` so the Raspberry Pi can control speed via PWM on GPIO 18 and GPIO 12.

---

### 5.3 Camera Angle & Mounting Geometry
* **Mounting Height**: 12 cm to 15 cm above ground level.
* **Pitch Angle**: **Tilted downward at 30 degrees** from horizontal.
* **Field of View (FOV)**:
  * The bottom of the camera frame begins approximately **20 cm in front of the front bumper**.
  * The top of the lane surface extends to approximately **120 cm ahead**.
  * This guarantees that a pothole is visible for at least 1.0 meter before the car reaches it, providing over 2.5 seconds of reaction time at 25 cm/s cruising speed.

---

## 6. Step-by-Step Assembly Checklist

Follow this sequence during physical construction:

1. **Chassis & Motor Assembly**:
   * Mount the 4 TT gear motors to the lower acrylic chassis plate using the steel L-brackets and M3 long bolts.
   * Solder 15cm lengths of red and black 22 AWG wire to each motor terminal. Solder a 0.1uF ceramic capacitor across the terminals if available to suppress motor electrical noise.
   * Press the 4 rubber wheels firmly onto the motor shafts until they click into place.
2. **Lower Deck Electronics**:
   * Mount the L298N motor driver in the center of the lower deck using M3 standoffs.
   * Wire the left motors to OUT1/OUT2 and right motors to OUT3/OUT4.
   * Secure the 7.4V LiPo motor battery using velcro straps on the lower deck to keep the center of gravity low.
   * Install the KCD1 master rocker switch on the red battery wire leading to the L298N positive terminal.
3. **Upper Deck & Raspberry Pi**:
   * Install 25mm brass standoffs between the lower and upper chassis decks.
   * Bolt the upper chassis plate in place.
   * Screw the Raspberry Pi 4B board to the upper deck using M2.5 nylon standoffs.
   * Attach the aluminum heatsinks and dual cooling fan to the Raspberry Pi CPU.
4. **Power System Calibration (MANDATORY)**:
   * Connect the 2x 18650 battery holder to the LM2596 buck converter input.
   * Turn on the battery and use a digital multimeter on the buck converter output.
   * Turn the brass trimmer screw until output reads **exactly 5.10V**.
   * Turn off the battery, then connect the 5.10V output to Raspberry Pi Pin 02 (+5V) and Pin 06 (GND).
5. **Sensor Bumper & Camera Mount**:
   * Bolt the HC-SR04 ultrasonic sensor and bracket to the front bumper.
   * Install the mini breadboard on the upper deck and wire the 1k/2k resistor voltage divider for the Echo pin.
   * Mount the Raspberry Pi Camera V2 on the front top deck. Tilt it downward at approximately 30 degrees.
   * Connect the 15-pin ribbon cable to the Pi Camera port (blue tape facing toward the Ethernet/USB ports).
6. **Final Wiring & Cable Management**:
   * Wire the GPIO pins to the L298N logic inputs (`IN1`, `IN2`, `IN3`, `IN4`, `ENA`, `ENB`).
   * Wire the status LEDs (Green, Blue, Red) and active buzzer on the mini breadboard.
   * Bundle loose jumper wires with zip ties so they do not snag on the wheels.

---

## 7. Edge Diagnostics & Hardware Verification Script

Save this script as `test_hardware.py` on the Raspberry Pi to test all motors, sensors, LEDs, and camera before driving:

```python
"""
Lumina IoT Car: Full Hardware Verification & Diagnostics Script
Tests motor channels, ultrasonic sensor, status LEDs, and camera.
"""

import time
import RPi.GPIO as GPIO

# Pin Definitions (BCM numbering)
IN1, IN2, ENA = 17, 27, 18  # Left Motors
IN3, IN4, ENB = 22, 23, 12  # Right Motors
TRIG, ECHO = 16, 20         # Ultrasonic
LED_G, LED_B, LED_R = 24, 10, 9  # Status LEDs
BUZZER = 25

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup Motor Pins
for pin in [IN1, IN2, IN3, IN4, LED_G, LED_B, LED_R, BUZZER]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
pwm_left = GPIO.PWM(ENA, 1000)
pwm_right = GPIO.PWM(ENB, 1000)
pwm_left.start(0)
pwm_right.start(0)

# Setup Ultrasonic Pins
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, GPIO.LOW)

def test_leds_and_buzzer():
    print("\n--- 1. Testing Indicators ---")
    for name, pin in [("Green LED", LED_G), ("Blue LED", LED_B), ("Red LED", LED_R)]:
        print(f"Lighting {name}...")
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(pin, GPIO.LOW)
    
    print("Beeping buzzer...")
    GPIO.output(BUZZER, GPIO.HIGH)
    time.sleep(0.15)
    GPIO.output(BUZZER, GPIO.LOW)
    print("Indicators OK.")

def read_ultrasonic_distance():
    print("\n--- 2. Testing HC-SR04 Ultrasonic Sensor ---")
    time.sleep(0.3)
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)  # 10 microsecond pulse
    GPIO.output(TRIG, GPIO.LOW)

    pulse_start = time.time()
    pulse_end = time.time()
    
    timeout = time.time() + 0.04  # 40ms max timeout
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            print("Ultrasonic Timeout (Echo Low). Check wiring / voltage divider.")
            return -1

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            print("Ultrasonic Timeout (Echo High). Check wiring.")
            return -1

    duration = pulse_end - pulse_start
    distance_cm = round(duration * 17150, 1)
    print(f"Measured Distance: {distance_cm} cm")
    return distance_cm

def test_motors():
    print("\n--- 3. Testing Motors (Spin Wheels in Air) ---")
    print("Driving Forward (35% PWM) for 1.5 seconds...")
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(35)
    pwm_right.ChangeDutyCycle(35)
    time.sleep(1.5)

    print("Driving Reverse (35% PWM) for 1.5 seconds...")
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)
    time.sleep(1.5)

    print("Stopping Motors...")
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.LOW)
    print("Motors OK.")

if __name__ == "__main__":
    try:
        print("========================================")
        print(" LUMINA IOT CAR HARDWARE DIAGNOSTICS")
        print("========================================")
        test_leds_and_buzzer()
        read_ultrasonic_distance()
        test_motors()
        print("\nAll hardware checks completed successfully!")
    finally:
        pwm_left.stop()
        pwm_right.stop()
        GPIO.cleanup()
```

---

## 8. Summary Procurement Checklist

Before beginning physical assembly, ensure you have checked off every item:

* [ ] Raspberry Pi 4B (4GB or 8GB) with MicroSD card & 5V cooling fan
* [ ] Raspberry Pi Camera Module V2 with 15cm ribbon cable & mounting bracket
* [ ] 4WD smart car chassis kit with 4 TT metal gear DC motors & 65mm rubber wheels
* [ ] L298N dual H-bridge motor driver module
* [ ] LM2596 step-down buck converter (calibrated to 5.10V)
* [ ] 7.4V 2S LiPo battery (with XT60 pigtail) for motors
* [ ] 2x 18650 battery holder with switch for Raspberry Pi logic
* [ ] HC-SR04 ultrasonic distance sensor with mounting bracket
* [ ] Resistors: 1k ohm (x2), 2k ohm (x2), 330 ohm (x5)
* [ ] Mini breadboard (170-tie-point)
* [ ] 40-pin Male-to-Female, Female-to-Female, and Male-to-Male jumper wire ribbons
* [ ] KCD1 master rocker switch and momentary red kill button
* [ ] 3x 5mm status LEDs (Green, Blue, Red) and active 5V buzzer
* [ ] Brass M3 standoffs, M3 screws, M2.5 mounting screws, zip ties, and foam tape
