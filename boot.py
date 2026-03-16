# @file boot.py
# @brief MicroPython boot configuration for the MECHA16 Romi
# @author Dylan-WALTY
# @date 2026-03-16

# This file is used to configure the boot process of MicroPython on the MECHA16 Romi platform.
# Any initialization required for the robot should be handled here.

import machine
import time

# Pin configuration and initialization
led = machine.Pin(25, machine.Pin.OUT)

# Function to flash the onboard LED
def flash_led():
    for _ in range(5):
        led.on()
        time.sleep(0.2)
        led.off()
        time.sleep(0.2)

# Flash the LED on boot
flash_led()
