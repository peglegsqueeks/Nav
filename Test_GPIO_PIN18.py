#!/usr/bin/env python3
"""
GPIO Pin 18 Brake Test Program for Jetson Orin NX
Tests brake relay GPIO pin operation

Run with: python3 test_gpio_pin18.py
"""

import time
import Jetson.GPIO as GPIO

PIN = 18  # BOARD pin numbering

print("=" * 50)
print("GPIO PIN 18 BRAKE TEST")
print("=" * 50)

# Setup
print("\n[1] Setting up Pin 18 as OUTPUT...")
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(PIN, GPIO.OUT)
print("    Done")

# Test LOW (Brake ON)
print("\n[2] Setting Pin 18 LOW...")
GPIO.output(PIN, GPIO.LOW)
time.sleep(0.5)
print("    BRAKE should be ON (relay de-energized)")
time.sleep(4)

# Test HIGH (Brake OFF)
print("\n[3] Setting Pin 18 HIGH...")
GPIO.output(PIN, GPIO.HIGH)
time.sleep(0.5)
print("    BRAKE should be OFF (relay energized)")
time.sleep(10)

# Return to LOW (Brake ON)
print("\n[4] Returning Pin 18 to LOW...")
GPIO.output(PIN, GPIO.LOW)
time.sleep(0.5)
print("    BRAKE should be ON (relay de-energized)")

print("\n" + "=" * 50)
print("TEST COMPLETE - Pin 18 left LOW (Brake ON)")
print("=" * 50)

GPIO.cleanup()