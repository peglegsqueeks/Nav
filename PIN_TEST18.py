#!/usr/bin/env python3
"""
Pin 18 Diagnostic Test with State Reading
"""
import time
import Jetson.GPIO as GPIO

PIN = 18

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(PIN, GPIO.OUT)

def read_pin_state():
    """Read current pin state"""
    GPIO.setup(PIN, GPIO.IN)
    state = GPIO.input(PIN)
    GPIO.setup(PIN, GPIO.OUT)
    return state

print("=" * 50)
print("PIN 18 DIAGNOSTIC TEST WITH STATE READING")
print("=" * 50)

# Test 1: Set LOW
print("\n[1] Setting LOW...")
GPIO.output(PIN, GPIO.LOW)
time.sleep(0.3)
state = read_pin_state()
GPIO.output(PIN, GPIO.LOW)  # Restore after read
print(f"    Written: LOW")
print(f"    Read:    {'HIGH' if state else 'LOW'} (raw: {state})")
print(f"    Match:   {'YES' if state == 0 else 'NO!'}")
print("    Waiting 5 seconds...")
time.sleep(5)

# Test 2: Set HIGH
print("\n[2] Setting HIGH...")
GPIO.output(PIN, GPIO.HIGH)
time.sleep(0.3)
state = read_pin_state()
GPIO.output(PIN, GPIO.HIGH)  # Restore after read
print(f"    Written: HIGH")
print(f"    Read:    {'HIGH' if state else 'LOW'} (raw: {state})")
print(f"    Match:   {'YES' if state == 1 else 'NO!'}")
print("    Waiting 5 seconds...")
time.sleep(5)

# Test 3: Set LOW again
print("\n[3] Setting LOW...")
GPIO.output(PIN, GPIO.LOW)
time.sleep(0.3)
state = read_pin_state()
GPIO.output(PIN, GPIO.LOW)  # Restore after read
print(f"    Written: LOW")
print(f"    Read:    {'HIGH' if state else 'LOW'} (raw: {state})")
print(f"    Match:   {'YES' if state == 0 else 'NO!'}")

print("\n" + "=" * 50)
print("TEST COMPLETE")
print("=" * 50)

GPIO.cleanup()