#!/usr/bin/env python3
"""
Diagnostic test for ADS1115
Tests I2C communication and input states
"""

import time
import smbus2

ADS1115_ADDRESS = 0x48
ADS1115_BUS = 3
REG_CONFIG = 0x01
REG_CONVERSION = 0x00

bus = smbus2.SMBus(ADS1115_BUS)

print("="*60)
print("ADS1115 Diagnostic Test")
print("="*60)

# Test 1: Read config register multiple times
print("\nTest 1: Reading config register 5 times (should be consistent)")
for i in range(5):
    data = bus.read_i2c_block_data(ADS1115_ADDRESS, REG_CONFIG, 2)
    config = (data[0] << 8) | data[1]
    print(f"  Read #{i+1}: 0x{config:04X}")
    time.sleep(0.1)

# Test 2: Ground one input and read it
print("\n" + "="*60)
print("Test 2: Measuring floating vs grounded inputs")
print("="*60)
print("\nInstructions:")
print("  1. Currently all inputs are floating (no connections)")
print("  2. Connect A0 to GND using a jumper wire")
print("  3. Watch the readings stabilize\n")

input("Press Enter when ready to start reading...")

from TransducerTest import ADS1115, CONFIG_PGA_4_096V

adc = ADS1115(bus_num=ADS1115_BUS, address=ADS1115_ADDRESS, gain=CONFIG_PGA_4_096V)

print("\nReading A0 (should show ~0V when grounded):")
print("Press Ctrl+C to stop\n")

try:
    for i in range(20):
        voltage = adc.read_voltage(0)
        adc_raw = adc.read_adc(0)
        print(f"  Sample {i+1:2d}: {adc_raw:6d} LSB → {voltage:6.3f}V")
        time.sleep(0.5)
except KeyboardInterrupt:
    pass

adc.close()