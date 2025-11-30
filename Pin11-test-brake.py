#!/usr/bin/env python3
"""
GPIO Pin 11 Test Program for Jetson Orin NX
Run with: python3 test_gpio_pin11.py
"""

import time
import Jetson.GPIO as GPIO

PIN = 11

print("=" * 40)
print("GPIO Pin 11 Test")
print("=" * 40)



# Setup
print("\n[2] Setting up Pin 11 as OUTPUT...")
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(PIN, GPIO.OUT)
print("    Done")

# Test LOW
print("\n[3] Setting Pin 11 LOW...")
GPIO.output(PIN, GPIO.LOW)
time.sleep(0.5)
state = GPIO.input(PIN)
print(f"    State read: {state} ({'HIGH' if state else 'LOW'})")
print("    BRAKE should be ON")
time.sleep(2)

# Test HIGH
print("\n[4] Setting Pin 11 HIGH...")
GPIO.output(PIN, GPIO.HIGH)
time.sleep(0.5)
state = GPIO.input(PIN)
print(f"    State read: {state} ({'HIGH' if state else 'LOW'})")
print("    BRAKE should be OFF")
time.sleep(2)

# Return to LOW
print("\n[5] Returning Pin 11 to LOW...")
GPIO.output(PIN, GPIO.LOW)
time.sleep(0.5)
state = GPIO.input(PIN)
print(f"    State read: {state} ({'HIGH' if state else 'LOW'})")
print("    BRAKE should be ON")

print("\n" + "=" * 40)
print("TEST COMPLETE - Pin 11 left LOW")
print("=" * 40)
# Reset GPIO
print("\n[1] Resetting GPIO...")
GPIO.cleanup()
print("    Done")