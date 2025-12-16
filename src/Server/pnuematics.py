import RPi.GPIO as GPIO
import time
from typing import Dict, List, Mapping, Optional, Tuple


# ============================================
# Configuration
# ============================================

# Valve names/locations (customize for your robot)
VALVE_NAMES = {
    1: "Front Left",
    2: "Front Right",
    3: "Back Left",
    4: "Back Right",
}

# GPIO pin assignments (BCM numbering)
DEFAULT_VALVE_PINS: Mapping[int, int] = {
    1: 13,  # Front Left
    2: 19,  # Front Right
    3: 16,  # Back Left
    4: 20,  # Back Right
}

class PnuematicsController:
    def __init__(self, valve_pins=DEFAULT_VALVE_PINS):
        self.valve_pins = dict(valve_pins)
        self.setup_gpio()
    # ============================================
    # GPIO Setup
    # ============================================

    def setup_gpio(self):
        """Initialize GPIO pins for solenoid control"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for valve_num, pin in self.valve_pins.items():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)  # Start with all valves closed
        
        print("✓ GPIO initialized - all valves closed")

    def cleanup_gpio(self):
        """Clean up GPIO - close all valves"""
        for pin in self.valve_pins.values():
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()
        print("✓ GPIO cleaned up - all valves closed")

    # ============================================
    # Valve Control Functions
    # ============================================

    def open_valve(self, valve_num):
        """
        Open specified valve
        
        Args:
            valve_num: Valve number (1-4)
        """
        if valve_num not in self.valve_pins:
            print(f"✗ Invalid valve number: {valve_num}")
            return
        
        pin = self.valve_pins[valve_num]
        GPIO.output(pin, GPIO.HIGH)
        print(f"✓ Valve {valve_num} ({VALVE_NAMES[valve_num]}) OPENED")

    def close_valve(self, valve_num):
        """
        Close specified valve
        
        Args:
            valve_num: Valve number (1-4)
        """
        if valve_num not in self.valve_pins:
            print(f"✗ Invalid valve number: {valve_num}")
            return
        
        pin = self.valve_pins[valve_num]
        GPIO.output(pin, GPIO.LOW)
        print(f"✓ Valve {valve_num} ({VALVE_NAMES[valve_num]}) CLOSED")

    def close_all_valves(self):
        """Close all valves"""
        for pin in self.valve_pins.values():
            GPIO.output(pin, GPIO.LOW)
        print("✓ All valves CLOSED")

    def open_all_valves(self):
        """Open all valves"""
        for pin in self.valve_pins.values():
            GPIO.output(pin, GPIO.HIGH)
        print("✓ All valves OPENED")

    def get_valve_state(self, valve_num):
        """
        Get current state of valve
        
        Args:
            valve_num: Valve number (1-4)
            
        Returns:
            True if open, False if closed
        """
        if valve_num not in self.valve_pins:
            return False
        
        pin = self.valve_pins[valve_num]
        return GPIO.input(pin) == GPIO.HIGH
