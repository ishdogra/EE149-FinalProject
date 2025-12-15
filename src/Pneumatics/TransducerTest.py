#!/usr/bin/env python3
"""
ADS1115 Pressure Transducer Reader
Uses SMbus for software I2C on bus 3
Based on community best practices
"""

import time
import smbus2

# ============================================
# ADS1115 Constants (from datasheet)
# ============================================

# I2C Configuration
ADS1115_ADDRESS = 0x48
ADS1115_BUS = 3  # Our software I2C bus

# Register Addresses
REG_CONVERSION = 0x00  # Conversion register (read)
REG_CONFIG = 0x01      # Config register (write)

# Config Register Bits
# Bit 15: Start single conversion (1)
# Bits 14-12: Input multiplexer
CONFIG_MUX_AIN0 = 0x4000  # AIN0 vs GND
CONFIG_MUX_AIN1 = 0x5000  # AIN1 vs GND  
CONFIG_MUX_AIN2 = 0x6000  # AIN2 vs GND
CONFIG_MUX_AIN3 = 0x7000  # AIN3 vs GND

# Bits 11-9: Programmable Gain Amplifier
CONFIG_PGA_6_144V = 0x0000  # ±6.144V
CONFIG_PGA_4_096V = 0x0200  # ±4.096V (default)
CONFIG_PGA_2_048V = 0x0400  # ±2.048V
CONFIG_PGA_1_024V = 0x0600  # ±1.024V
CONFIG_PGA_0_512V = 0x0800  # ±0.512V
CONFIG_PGA_0_256V = 0x0A00  # ±0.256V

# Bit 8: Operating mode (1 = single-shot)
CONFIG_MODE_SINGLE = 0x0100

# Bits 7-5: Data rate
CONFIG_DR_128SPS = 0x0080  # 128 samples/sec (default)
CONFIG_DR_250SPS = 0x00A0  # 250 samples/sec
CONFIG_DR_860SPS = 0x00E0  # 860 samples/sec

# Bits 4-0: Comparator settings (disabled)
CONFIG_COMP_DIS = 0x0003

# ============================================
# ADS1115 Class
# ============================================

class ADS1115:
    """
    ADS1115 16-bit ADC driver using SMbus
    Suitable for software I2C buses
    """
    
    def __init__(self, bus_num=1, address=0x48, gain=CONFIG_PGA_4_096V):
        """
        Initialize ADS1115
        
        Args:
            bus_num: I2C bus number (default 1, use 3 for software I2C)
            address: I2C address (default 0x48)
            gain: PGA gain setting (default ±4.096V)
        """
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self.gain = gain
        
        # Calculate voltage per bit based on gain
        self.voltage_ranges = {
            CONFIG_PGA_6_144V: 6.144,
            CONFIG_PGA_4_096V: 4.096,
            CONFIG_PGA_2_048V: 2.048,
            CONFIG_PGA_1_024V: 1.024,
            CONFIG_PGA_0_512V: 0.512,
            CONFIG_PGA_0_256V: 0.256,
        }
        self.volts_per_bit = self.voltage_ranges[gain] / 32768.0
        
    def read_adc(self, channel):
        """
        Read ADC value from specified channel
        
        Args:
            channel: 0-3 for AIN0-AIN3
            
        Returns:
            Raw ADC value (signed 16-bit integer)
        """
        # Select input multiplexer
        mux_configs = {
            0: CONFIG_MUX_AIN0,
            1: CONFIG_MUX_AIN1,
            2: CONFIG_MUX_AIN2,
            3: CONFIG_MUX_AIN3,
        }
        
        if channel not in mux_configs:
            raise ValueError(f"Channel must be 0-3, got {channel}")
        
        # Build config word
        config = (0x8000 |                    # Start conversion
                  mux_configs[channel] |       # Input mux
                  self.gain |                  # PGA gain
                  CONFIG_MODE_SINGLE |         # Single-shot mode
                  CONFIG_DR_128SPS |           # 128 samples/sec
                  CONFIG_COMP_DIS)             # Disable comparator
        
        # Write config (big-endian, MSB first)
        config_bytes = [(config >> 8) & 0xFF, config & 0xFF]
        self.bus.write_i2c_block_data(self.address, REG_CONFIG, config_bytes)
        
        # Wait for conversion (8ms for 128 SPS)
        time.sleep(0.01)
        
        # Read conversion result
        data = self.bus.read_i2c_block_data(self.address, REG_CONVERSION, 2)
        
        # Convert to signed 16-bit integer
        value = (data[0] << 8) | data[1]
        if value > 32767:
            value -= 65536
            
        return value
    
    def read_voltage(self, channel):
        """
        Read voltage from specified channel
        
        Args:
            channel: 0-3 for AIN0-AIN3
            
        Returns:
            Voltage in volts
        """
        adc_value = self.read_adc(channel)
        return adc_value * self.volts_per_bit
    
    def close(self):
        """Close I2C bus"""
        self.bus.close()

# ============================================
# Pressure Conversion Functions
# ============================================

def voltage_to_pressure(voltage):
    """
    Converts voltage to pressure
    
    """
    kpa = (0.75 * (voltage - 0.5) - 1) * 100
    
    # For vacuum, pressure will be negative
    # Return absolute value for vacuum kPa
    return abs(kpa) if kpa < 0 else 0

# ============================================
# Main Test Program
# ============================================

def main():
    print("\n" + "="*60)
    print("ADS1115 Pressure Transducer Test (SMbus)")
    print("="*60)
    print(f"\nI2C Bus: {ADS1115_BUS}")
    print(f"Device Address: 0x{ADS1115_ADDRESS:02X}")
    print(f"Gain: ±4.096V")
    print("\nReading 4 pressure sensors on channels A0, A1, A2, A3")
    print("Press Ctrl+C to stop\n")
    
    # Initialize ADS1115
    try:
        adc = ADS1115(bus_num=ADS1115_BUS, 
                      address=ADS1115_ADDRESS,
                      gain=CONFIG_PGA_4_096V)
    except Exception as e:
        print(f"Error initializing ADS1115: {e}")
        print("\nTroubleshooting:")
        print("  1. Check wiring (SDA=GPIO5, SCL=GPIO6)")
        print("  2. Verify pull-up resistors (4.7kΩ)")
        print("  3. Run: sudo i2cdetect -y 3")
        print("  4. Check /boot/firmware/config.txt for i2c-gpio overlay")
        return
    
    # Define sensors
    sensors = [
        {"channel": 0, "name": "Sensor 1 (A0)", "location": "Front Left"},
        {"channel": 1, "name": "Sensor 2 (A1)", "location": "Front Right"},
        {"channel": 2, "name": "Sensor 3 (A2)", "location": "Back Left"},
        {"channel": 3, "name": "Sensor 4 (A3)", "location": "Back Right"},
    ]
    
    try:
        iteration = 0
        while True:
            print(f"\n{'─' * 60}")
            print(f"Reading #{iteration+1:4d} - {time.strftime('%H:%M:%S')}")
            print(f"{'─' * 60}")
            
            all_voltages = []
            all_pressures = []
            
            for sensor in sensors:
                try:
                    # Read raw ADC value
                    adc_raw = adc.read_adc(sensor['channel'])
                    
                    # Convert to voltage
                    voltage = adc.read_voltage(sensor['channel']) * 3.128
                    
                    # Convert to pressure
                    pressure = voltage_to_pressure(voltage)
                    
                    all_voltages.append(voltage)
                    all_pressures.append(pressure)
                    
                    # Display
                    print(f"{sensor['name']:16s} ({sensor['location']:12s}): "
                          f"{adc_raw:6d} LSB  →  {voltage:5.3f}V  →  {pressure:6.1f} kPa")
                    
                except Exception as e:
                    print(f"{sensor['name']:16s}: Error - {e}")
            
            # Display summary
            if all_pressures:
                avg_pressure = sum(all_pressures) / len(all_pressures)
                min_pressure = min(all_pressures)
                max_pressure = max(all_pressures)
                
                print(f"\n{'─' * 60}")
                print(f"Average: {avg_pressure:6.1f} kPa  |  "
                      f"Range: {min_pressure:6.1f} - {max_pressure:6.1f} kPa")
            
            iteration += 1
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("Stopped by user")
        print("="*60)
    finally:
        adc.close()

if __name__ == "__main__":
    main()