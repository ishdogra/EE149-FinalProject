#!/usr/bin/env python3
"""
Test SerLCD on software I2C bus 4
Uses SMbus for direct control
"""

import smbus2
import time

# ============================================
# Configuration
# ============================================

LCD_BUS = 4           # Our new I2C bus for LCD
LCD_ADDRESS = 0x72    # Default SerLCD address

# SerLCD Commands
CMD_SPECIAL = 0xFE
CMD_SETTING = 0x7C

# ============================================
# SerLCD Class
# ============================================

class SerLCD:
    """SerLCD control via SMbus"""
    
    def __init__(self, bus_num=4, address=0x72):
        """Initialize LCD on specified I2C bus"""
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        time.sleep(0.5)  # Wait for LCD to initialize
        self.clear()
    
    def write_bytes(self, data):
        """Write bytes to LCD"""
        if isinstance(data, int):
            data = [data]
        elif isinstance(data, str):
            data = [ord(c) for c in data]
        
        self.bus.write_i2c_block_data(self.address, data[0], data[1:])
        time.sleep(0.01)
    
    def clear(self):
        """Clear display"""
        self.write_bytes([CMD_SETTING, 0x2D])
        time.sleep(0.05)
    
    def set_cursor(self, col, row):
        """Set cursor position (col: 0-15, row: 0-1)"""
        position = col + (row * 64)
        self.write_bytes([CMD_SPECIAL, 0x80 + position])
    
    def set_rgb(self, r, g, b):
        """Set RGB backlight (r, g, b: 0-255)"""
        self.write_bytes([CMD_SETTING, 0x2B, r, g, b])
        time.sleep(0.05)
    
    def set_brightness(self, brightness):
        """Set backlight brightness (0-255)"""
        brightness = max(0, min(255, brightness))
        self.write_bytes([CMD_SETTING, 0x80 + brightness])
    
    def print_text(self, text):
        """Print text at current cursor position"""
        if len(text) > 0:
            # Split into chunks of 30 bytes for I2C transmission
            for i in range(0, len(text), 30):
                chunk = text[i:i+30]
                data = [ord(c) for c in chunk]
                if len(data) > 1:
                    self.bus.write_i2c_block_data(self.address, data[0], data[1:])
                else:
                    self.bus.write_byte(self.address, data[0])
                time.sleep(0.01)
    
    def print_line(self, row, text):
        """Print text on specified line (clears line first)"""
        text = text[:16].ljust(16)  # Pad to 16 chars
        self.set_cursor(0, row)
        self.print_text(text)
    
    def close(self):
        """Close I2C bus"""
        self.bus.close()

# ============================================
# Test Functions
# ============================================

def test_basic_display():
    """Test basic text display"""
    print("\n" + "="*60)
    print("Test 1: Basic Display")
    print("="*60)
    
    lcd = SerLCD(bus_num=LCD_BUS, address=LCD_ADDRESS)
    
    lcd.set_rgb(0, 255, 0)  # Green
    lcd.print_line(0, "SerLCD Test")
    lcd.print_line(1, "Bus 4 Working!")
    
    print("Display should show:")
    print("  Line 1: 'SerLCD Test'")
    print("  Line 2: 'Bus 4 Working!'")
    print("  Color: Green")
    
    time.sleep(3)
    lcd.close()

def test_color_cycle():
    """Test RGB color cycling"""
    print("\n" + "="*60)
    print("Test 2: Color Cycle")
    print("="*60)
    
    lcd = SerLCD(bus_num=LCD_BUS, address=LCD_ADDRESS)
    
    colors = [
        (255, 0, 0, "Red"),
        (0, 255, 0, "Green"),
        (0, 0, 255, "Blue"),
        (255, 255, 0, "Yellow"),
        (255, 0, 255, "Magenta"),
        (0, 255, 255, "Cyan"),
        (255, 255, 255, "White"),
    ]
    
    for r, g, b, name in colors:
        lcd.set_rgb(r, g, b)
        lcd.print_line(0, "Color Test")
        lcd.print_line(1, name.center(16))
        print(f"  Color: {name}")
        time.sleep(1)
    
    lcd.close()

def test_counter():
    """Test dynamic updates"""
    print("\n" + "="*60)
    print("Test 3: Counter (Press Ctrl+C to stop)")
    print("="*60)
    
    lcd = SerLCD(bus_num=LCD_BUS, address=LCD_ADDRESS)
    lcd.set_rgb(0, 255, 0)
    
    try:
        count = 0
        while True:
            lcd.print_line(0, "Counter Test")
            lcd.print_line(1, f"Count: {count}".center(16))
            print(f"  Count: {count}")
            count += 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n  Stopped by user")
    
    lcd.close()

def test_brightness():
    """Test backlight brightness"""
    print("\n" + "="*60)
    print("Test 4: Brightness Test")
    print("="*60)
    
    lcd = SerLCD(bus_num=LCD_BUS, address=LCD_ADDRESS)
    lcd.set_rgb(255, 255, 255)
    lcd.print_line(0, "Brightness Test")
    
    # Fade up
    print("  Fading up...")
    for brightness in range(0, 256, 16):
        lcd.set_brightness(brightness)
        lcd.print_line(1, f"Level: {brightness}".center(16))
        time.sleep(0.1)
    
    time.sleep(0.5)
    
    # Fade down
    print("  Fading down...")
    for brightness in range(255, -1, -16):
        lcd.set_brightness(brightness)
        lcd.print_line(1, f"Level: {brightness}".center(16))
        time.sleep(0.1)
    
    # Restore brightness
    lcd.set_brightness(255)
    
    lcd.close()

# ============================================
# Main Program
# ============================================

def main():
    print("\n" + "="*60)
    print("SerLCD Test Program - I2C Bus 4")
    print("="*60)
    print(f"\nConfiguration:")
    print(f"  Bus: {LCD_BUS}")
    print(f"  Address: 0x{LCD_ADDRESS:02X}")
    print(f"  GPIO SDA: 21 (Pin 40)")
    print(f"  GPIO SCL: 26 (Pin 37)")
    
    # Check if LCD is detected
    print("\nScanning I2C bus...")
    try:
        bus = smbus2.SMBus(LCD_BUS)
        
        # Try to read from LCD
        try:
            bus.read_byte(LCD_ADDRESS)
            print(f"✓ SerLCD detected at 0x{LCD_ADDRESS:02X}")
        except:
            print(f"✗ SerLCD not found at 0x{LCD_ADDRESS:02X}")
            print("\nTroubleshooting:")
            print("  1. Check wiring (SDA=GPIO21, SCL=GPIO26)")
            print("  2. Run: sudo i2cdetect -y 4")
            print("  3. Verify bus 4 exists: ls /dev/i2c-4")
            bus.close()
            return
        
        bus.close()
        
    except Exception as e:
        print(f"✗ Error accessing I2C bus {LCD_BUS}: {e}")
        print("\nTroubleshooting:")
        print("  1. Verify overlay in /boot/firmware/config.txt")
        print("  2. Check: ls /dev/i2c-4")
        print("  3. Reboot if you just added the overlay")
        return
    
    print("\n" + "="*60)
    input("Press Enter to start tests...")
    
    try:
        test_basic_display()
        time.sleep(1)
        
        test_color_cycle()
        time.sleep(1)
        
        test_brightness()
        time.sleep(1)
        
        test_counter()
        
    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
    except Exception as e:
        print(f"\n\nError during test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up - turn off display
        try:
            lcd = SerLCD(bus_num=LCD_BUS, address=LCD_ADDRESS)
            lcd.clear()
            lcd.set_rgb(0, 0, 0)
            lcd.close()
        except:
            pass
    
    print("\n" + "="*60)
    print("Tests Complete")
    print("="*60)

if __name__ == "__main__":
    main()