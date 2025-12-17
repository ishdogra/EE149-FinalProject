#!/usr/bin/env python3
"""
Solenoid valve test script
Controls 4 solenoid valves via GPIO
"""

import RPi.GPIO as GPIO
import time

# ============================================
# Configuration
# ============================================

# GPIO pin assignments (BCM numbering)
VALVE_PINS = {
    1: 13,  # Solenoid 1 → GPIO 12 (Pin 32)
    2: 19,  # Solenoid 2 → GPIO 13 (Pin 33)
    3: 16,  # Solenoid 3 → GPIO 16 (Pin 36)
    4: 20,  # Solenoid 4 → GPIO 19 (Pin 35)
}

# Valve names/locations (customize for your robot)
VALVE_NAMES = {
    1: "Front Right",
    2: "Back Right",
    3: "Back Left",
    4: "Front Left",
}

# ============================================
# GPIO Setup
# ============================================

def setup_gpio():
    """Initialize GPIO pins for solenoid control"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    for valve_num, pin in VALVE_PINS.items():
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)  # Start with all valves closed
    
    print("✓ GPIO initialized - all valves closed")

def cleanup_gpio():
    """Clean up GPIO - close all valves"""
    for pin in VALVE_PINS.values():
        GPIO.output(pin, GPIO.LOW)
    GPIO.cleanup()
    print("✓ GPIO cleaned up - all valves closed")

# ============================================
# Valve Control Functions
# ============================================

def open_valve(valve_num):
    """
    Open specified valve
    
    Args:
        valve_num: Valve number (1-4)
    """
    if valve_num not in VALVE_PINS:
        print(f"✗ Invalid valve number: {valve_num}")
        return
    
    pin = VALVE_PINS[valve_num]
    GPIO.output(pin, GPIO.HIGH)
    print(f"✓ Valve {valve_num} ({VALVE_NAMES[valve_num]}) OPENED")

def close_valve(valve_num):
    """
    Close specified valve
    
    Args:
        valve_num: Valve number (1-4)
    """
    if valve_num not in VALVE_PINS:
        print(f"✗ Invalid valve number: {valve_num}")
        return
    
    pin = VALVE_PINS[valve_num]
    GPIO.output(pin, GPIO.LOW)
    print(f"✓ Valve {valve_num} ({VALVE_NAMES[valve_num]}) CLOSED")

def close_all_valves():
    """Close all valves"""
    for pin in VALVE_PINS.values():
        GPIO.output(pin, GPIO.LOW)
    print("✓ All valves CLOSED")

def open_all_valves():
    """Open all valves"""
    for pin in VALVE_PINS.values():
        GPIO.output(pin, GPIO.HIGH)
    print("✓ All valves OPENED")

def get_valve_state(valve_num):
    """
    Get current state of valve
    
    Args:
        valve_num: Valve number (1-4)
        
    Returns:
        True if open, False if closed
    """
    if valve_num not in VALVE_PINS:
        return False
    
    pin = VALVE_PINS[valve_num]
    return GPIO.input(pin) == GPIO.HIGH

# ============================================
# Test Functions
# ============================================

def test_individual_valves():
    """Test each valve individually"""
    print("\n" + "="*60)
    print("Test 1: Individual Valve Test")
    print("="*60)
    print("Opening each valve for 2 seconds...\n")
    
    for valve_num in sorted(VALVE_PINS.keys()):
        open_valve(valve_num)
        time.sleep(10)
        close_valve(valve_num)
        time.sleep(0.5)
    
    print("\n✓ Individual valve test complete")

def test_all_valves():
    """Test all valves together"""
    print("\n" + "="*60)
    print("Test 2: All Valves Test")
    print("="*60)
    print("Opening all valves for 3 seconds...\n")
    
    open_all_valves()
    time.sleep(3)
    close_all_valves()
    
    print("\n✓ All valves test complete")

def test_sequence():
    """Test valves in sequence"""
    print("\n" + "="*60)
    print("Test 3: Sequential Pattern Test")
    print("="*60)
    print("Running wave pattern...\n")
    
    # Forward wave
    for valve_num in sorted(VALVE_PINS.keys()):
        open_valve(valve_num)
        time.sleep(0.5)
    
    time.sleep(1)
    
    # Reverse wave
    for valve_num in sorted(VALVE_PINS.keys(), reverse=True):
        close_valve(valve_num)
        time.sleep(0.5)
    
    print("\n✓ Sequential test complete")

def test_pulse():
    """Test rapid on/off pulsing"""
    print("\n" + "="*60)
    print("Test 4: Pulse Test")
    print("="*60)
    print("Pulsing valve 1 rapidly (5 cycles)...\n")
    
    for i in range(5):
        print(f"  Pulse {i+1}/5")
        open_valve(1)
        time.sleep(0.2)
        close_valve(1)
        time.sleep(0.2)
    
    print("\n✓ Pulse test complete")

def interactive_mode():
    """Interactive valve control"""
    print("\n" + "="*60)
    print("Interactive Mode")
    print("="*60)
    print("\nCommands:")
    print("  1-4  : Toggle valve 1-4")
    print("  a    : Open all valves")
    print("  c    : Close all valves")
    print("  s    : Show valve states")
    print("  q    : Quit")
    print()
    
    while True:
        try:
            cmd = input("Command> ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'a':
                open_all_valves()
            elif cmd == 'c':
                close_all_valves()
            elif cmd == 's':
                print("\nValve States:")
                for valve_num in sorted(VALVE_PINS.keys()):
                    state = "OPEN" if get_valve_state(valve_num) else "CLOSED"
                    print(f"  Valve {valve_num} ({VALVE_NAMES[valve_num]:12s}): {state}")
                print()
            elif cmd.isdigit() and 1 <= int(cmd) <= 6:
                valve_num = int(cmd)
                if get_valve_state(valve_num):
                    close_valve(valve_num)
                else:
                    open_valve(valve_num)
            else:
                print("Invalid command")
                
        except KeyboardInterrupt:
            break
    
    print("\n✓ Exiting interactive mode")

# ============================================
# Main Program
# ============================================

def main():
    print("\n" + "="*60)
    print("Solenoid Valve Test Program")
    print("="*60)
    print(f"\nConfigured for {len(VALVE_PINS)} valves")
    print("\nGPIO Pin Mapping:")
    for valve_num in sorted(VALVE_PINS.keys()):
        pin = VALVE_PINS[valve_num]
        name = VALVE_NAMES[valve_num]
        print(f"  Valve {valve_num} ({name:12s}) → GPIO {pin:2d} (Pin {get_physical_pin(pin):2d})")
    
    print("\n" + "="*60)
    print("SAFETY CHECK")
    print("="*60)
    print("\n⚠ WARNING: Ensure proper wiring before proceeding!")
    print("\nRequired for each valve:")
    print("  ✓ Flyback diode (1N4007) across solenoid")
    print("  ✓ Transistor/MOSFET properly rated")
    print("  ✓ Common ground between Pi and 12V supply")
    print("  ✓ 12V power supply connected")
    print()
    
    response = input("Ready to proceed? (yes/no): ").strip().lower()
    if response != 'yes':
        print("Aborted by user")
        return
    
    # Initialize GPIO
    setup_gpio()
    
    try:
        # Run tests
        test_individual_valves()
        time.sleep(1)
        
        test_all_valves()
        time.sleep(1)
        
        test_sequence()
        time.sleep(1)
        
        test_pulse()
        time.sleep(1)
        
        # Interactive mode
        print("\n" + "="*60)
        response = input("Enter interactive mode? (yes/no): ").strip().lower()
        if response == 'yes':
            interactive_mode()
        
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
    
    finally:
        # Always clean up
        cleanup_gpio()
        print("\n" + "="*60)
        print("Test Complete")
        print("="*60)

def get_physical_pin(gpio_num):
    """Convert BCM GPIO number to physical pin number"""
    pin_map = {
        13: 33, 19: 35, 16: 36, 20: 38, 
        5: 29, 6: 31
    }
    return pin_map.get(gpio_num, 0)

if __name__ == "__main__":
    main()