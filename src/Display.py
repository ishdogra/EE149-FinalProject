#!/usr/bin/env python3
"""
Quadrant display for hexapod leg status
Shows 4 legs in quadrants on 16x2 LCD
"""

import smbus2
import time

class QuadrantDisplay:
    """
    Display class for showing 4 leg statuses on SerLCD
    
    Layout:
    ┌────────────────┐
    │ FL      FR     │  Line 0: Front Left | Front Right
    │ BL      BR     │  Line 1: Back Left  | Back Right
    └────────────────┘
    
    Each quadrant shows: "AT:50.2" or "DT: 0.0"
    """
    
    # LCD Commands
    CMD_SPECIAL = 0xFE
    CMD_SETTING = 0x7C
    
    # Leg positions
    FRONT_LEFT = 0
    FRONT_RIGHT = 1
    BACK_LEFT = 2
    BACK_RIGHT = 3
    
    def __init__(self, bus_num=4, address=0x72):
        """
        Initialize display
        
        Args:
            bus_num: I2C bus number (default 4)
            address: I2C address (default 0x72)
        """
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        
        # Leg states (valve status and pressure)
        self.leg_states = [
            {"attached": False, "pressure": 0.0},  # Front Left
            {"attached": False, "pressure": 0.0},  # Front Right
            {"attached": False, "pressure": 0.0},  # Back Left
            {"attached": False, "pressure": 0.0},  # Back Right
        ]
        
        time.sleep(0.5)
        self.clear()
        self.set_rgb(255, 255, 255)  # White backlight
    
    def _write_bytes(self, data):
        """Write bytes to LCD"""
        if isinstance(data, int):
            data = [data]
        elif isinstance(data, str):
            data = [ord(c) for c in data]
        
        if len(data) > 1:
            self.bus.write_i2c_block_data(self.address, data[0], data[1:])
        else:
            self.bus.write_byte(self.address, data[0])
        time.sleep(0.01)
    
    def clear(self):
        """Clear display"""
        self._write_bytes([self.CMD_SETTING, 0x2D])
        time.sleep(0.05)
    
    def set_cursor(self, col, row):
        """Set cursor position (col: 0-15, row: 0-1)"""
        position = col + (row * 64)
        self._write_bytes([self.CMD_SPECIAL, 0x80 + position])
    
    def set_rgb(self, r, g, b):
        """Set RGB backlight (r, g, b: 0-255)"""
        self._write_bytes([self.CMD_SETTING, 0x2B, r, g, b])
        time.sleep(0.05)
    
    def _print_text(self, text):
        """Print text at current cursor position"""
        if len(text) > 0:
            for i in range(0, len(text), 30):
                chunk = text[i:i+30]
                data = [ord(c) for c in chunk]
                if len(data) > 1:
                    self.bus.write_i2c_block_data(self.address, data[0], data[1:])
                else:
                    self.bus.write_byte(self.address, data[0])
                time.sleep(0.01)
    
    def update_leg(self, leg_position, attached, pressure_kpa):
        """
        Update individual leg status
        
        Args:
            leg_position: FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, or BACK_RIGHT
            attached: True if valve open (attached), False if closed (detached)
            pressure_kpa: Vacuum pressure in kPa (will be displayed as positive)
        """
        if 0 <= leg_position < 4:
            self.leg_states[leg_position] = {
                "attached": attached,
                "pressure": abs(pressure_kpa)  # Always positive
            }
    
    def _format_leg_status(self, attached, pressure_kpa):
        """
        Format leg status string (8 characters)
        
        Args:
            attached: True for AT, False for DT
            pressure_kpa: Pressure value
            
        Returns:
            8-character string like "AT:50.2 " or "DT: 0.0 "
        """
        status = "AT" if attached else "DT"
        
        # Round to tenths
        pressure = round(abs(pressure_kpa), 1)
        
        # Format with proper spacing (8 chars total)
        if pressure >= 100.0:
            # 3 digits: "AT:123.4"
            formatted = f"{status}:{pressure:5.1f}"
        elif pressure >= 10.0:
            # 2 digits: "AT:12.3 "
            formatted = f"{status}:{pressure:4.1f} "
        else:
            # 1 digit: "AT: 1.2 "
            formatted = f"{status}:{pressure:4.1f} "
        
        # Ensure exactly 8 characters
        return formatted[:8].ljust(8)
    
    def refresh(self):
        """
        Refresh entire display with current leg states
        
        Layout:
        Line 0: [Front Left (8 chars)][Front Right (8 chars)]
        Line 1: [Back Left  (8 chars)][Back Right  (8 chars)]
        """
        # Format each quadrant
        fl = self._format_leg_status(
            self.leg_states[self.FRONT_LEFT]["attached"],
            self.leg_states[self.FRONT_LEFT]["pressure"]
        )
        
        fr = self._format_leg_status(
            self.leg_states[self.FRONT_RIGHT]["attached"],
            self.leg_states[self.FRONT_RIGHT]["pressure"]
        )
        
        bl = self._format_leg_status(
            self.leg_states[self.BACK_LEFT]["attached"],
            self.leg_states[self.BACK_LEFT]["pressure"]
        )
        
        br = self._format_leg_status(
            self.leg_states[self.BACK_RIGHT]["attached"],
            self.leg_states[self.BACK_RIGHT]["pressure"]
        )
        
        # Build display lines
        line0 = fl + fr  # Front Left + Front Right
        line1 = bl + br  # Back Left + Back Right
        
        # Update display
        self.set_cursor(0, 0)
        self._print_text(line0)
        
        self.set_cursor(0, 1)
        self._print_text(line1)
    
    def update_all(self, front_left, front_right, back_left, back_right):
        """
        Update all legs at once
        
        Args:
            front_left: (attached, pressure_kpa) tuple
            front_right: (attached, pressure_kpa) tuple
            back_left: (attached, pressure_kpa) tuple
            back_right: (attached, pressure_kpa) tuple
        """
        self.update_leg(self.FRONT_LEFT, *front_left)
        self.update_leg(self.FRONT_RIGHT, *front_right)
        self.update_leg(self.BACK_LEFT, *back_left)
        self.update_leg(self.BACK_RIGHT, *back_right)
        self.refresh()
    
    def close(self):
        """Close I2C bus"""
        self.bus.close()


# ============================================
# Test Program
# ============================================

def test_display():
    """Test the quadrant display"""
    
    print("\n" + "="*60)
    print("Quadrant Display Test")
    print("="*60)
    print("\nDisplay Layout:")
    print("  ┌────────────────┐")
    print("  │ FL      FR     │  Line 0")
    print("  │ BL      BR     │  Line 1")
    print("  └────────────────┘")
    print("\nFL = Front Left, FR = Front Right")
    print("BL = Back Left,  BR = Back Right")
    print()
    
    display = QuadrantDisplay(bus_num=4, address=0x72)
    
    # Test 1: All detached, no pressure
    print("Test 1: All legs detached, no pressure")
    display.update_all(
        front_left=(False, 0.0),
        front_right=(False, 0.0),
        back_left=(False, 0.0),
        back_right=(False, 0.0)
    )
    time.sleep(2)
    
    # Test 2: Front Left attaches with 45.3 kPa
    print("Test 2: Front Left attaches (45.3 kPa)")
    display.update_leg(QuadrantDisplay.FRONT_LEFT, True, 45.3)
    display.refresh()
    time.sleep(2)
    
    # Test 3: All legs attach with different pressures
    print("Test 3: All legs attach with varying pressures")
    display.update_all(
        front_left=(True, 52.7),
        front_right=(True, 48.2),
        back_left=(True, 51.1),
        back_right=(True, 49.5)
    )
    time.sleep(2)
    
    # Test 4: Front legs detach
    print("Test 4: Front legs detach")
    display.update_leg(QuadrantDisplay.FRONT_LEFT, False, 0.0)
    display.update_leg(QuadrantDisplay.FRONT_RIGHT, False, 0.0)
    display.refresh()
    time.sleep(2)
    
    # Test 5: Mixed states
    print("Test 5: Mixed states")
    display.update_all(
        front_left=(True, 123.4),   # 3 digits
        front_right=(False, 5.6),   # 1 digit
        back_left=(False, 12.3),    # 2 digits
        back_right=(True, 98.7)     # 2 digits
    )
    time.sleep(2)
    
    # Test 6: Simulated vacuum cycle
    print("Test 6: Simulated vacuum cycle")
    for pressure in range(0, 60, 5):
        display.update_all(
            front_left=(True, pressure),
            front_right=(True, pressure + 2),
            back_left=(True, pressure + 1),
            back_right=(True, pressure + 3)
        )
        time.sleep(0.3)
    
    time.sleep(1)
    
    # Test 7: Walking simulation
    print("Test 7: Walking simulation (alternating legs)")
    for i in range(5):
        # Front left and back right attached
        display.update_all(
            front_left=(True, 50.0),
            front_right=(False, 0.0),
            back_left=(False, 0.0),
            back_right=(True, 50.0)
        )
        time.sleep(0.5)
        
        # Front right and back left attached
        display.update_all(
            front_left=(False, 0.0),
            front_right=(True, 50.0),
            back_left=(True, 50.0),
            back_right=(False, 0.0)
        )
        time.sleep(0.5)
    
    # Clean up
    print("\nTest complete!")
    display.clear()
    display.set_rgb(0, 0, 0)
    display.close()


if __name__ == "__main__":
    test_display()