#!/usr/bin/env python3
"""
Simple GPIO Pin 11 Brake Test
Sets pin 11 HIGH (brake OFF), waits 30 seconds, then sets LOW (brake ON)
Watch the relay light during this test.
"""
import time
import Jetson.GPIO as GPIO

PIN = 11

print("=" * 50)
print("GPIO PIN 11 BRAKE TEST")
print("=" * 50)

# Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.OUT)

print(f"Setting PIN {PIN} HIGH (brake OFF)...")
GPIO.output(PIN, GPIO.HIGH)
print("Relay light should be ON (red light visible)")
print("")
print("Waiting 30 seconds - watch for any changes...")
print("")

for i in range(30, 0, -1):
    print(f"  {i} seconds remaining...", end='\r')
    time.sleep(1)

print("\n")
print(f"Setting PIN {PIN} LOW (brake ON)...")
GPIO.output(PIN, GPIO.LOW)
print("Relay light should now be OFF")
print("")
print("Test complete.")
GPIO.cleanup()