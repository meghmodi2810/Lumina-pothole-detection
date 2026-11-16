#!/bin/bash
# LUMINA Autonomous IoT Car - GPIO 2 Boot Silence Script
# Permanently silences the active buzzer on GPIO 2 (Physical Pin 3) at boot

echo "====================================================================="
echo " LUMINA CAR: Silencing GPIO 2 (Pin 3) Active Buzzer at Power-On"
echo "====================================================================="

# 1. Immediately drive live pin LOW (0V)
echo "[1/2] Clamping live GPIO 2 to LOW (0V)..."
if command -v pinctrl &>/dev/null; then
    sudo pinctrl set 2 op dl
    echo "  ✓ Live GPIO 2 driven LOW via pinctrl."
elif command -v raspi-gpio &>/dev/null; then
    sudo raspi-gpio set 2 op dl
    echo "  ✓ Live GPIO 2 driven LOW via raspi-gpio."
else
    echo "  ! Note: Neither pinctrl nor raspi-gpio found. Will clamp via config.txt."
fi

# 2. Append bootloader config to config.txt
echo "[2/2] Updating /boot/firmware/config.txt or /boot/config.txt..."
CFG="/boot/firmware/config.txt"
if [ ! -f "$CFG" ]; then
    CFG="/boot/config.txt"
fi

if [ -f "$CFG" ]; then
    if grep -q "gpio=2=op,dl" "$CFG"; then
        echo "  ✓ 'gpio=2=op,dl' is already present in $CFG."
    else
        echo -e "\n# LUMINA BUZZER HARDWARE CLAMP (GPIO 2 / PIN 3)\ngpio=2=op,dl\ndtparam=i2c_arm=off" | sudo tee -a "$CFG" >/dev/null
        echo "  ✓ Successfully added gpio=2=op,dl and dtparam=i2c_arm=off to $CFG"
    fi
else
    echo "  ! Error: Could not locate config.txt on this system."
fi

echo "====================================================================="
echo " Done! The buzzer will no longer buzz continuously when plugged in."
echo "====================================================================="
