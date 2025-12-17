#!/usr/bin/env python3
"""
ADS1115 Pressure Transducer Reader
Uses SMbus for software I2C on bus 3
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

class Transducer:
    """
    ADS1115 16-bit ADC driver using SMbus
    Suitable for software I2C buses
    """
    
    def __init__(self, bus_num=ADS1115_BUS, 
                      address=ADS1115_ADDRESS,
                      gain=CONFIG_PGA_4_096V):
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
        return adc_value * self.volts_per_bit * 3.128 
    
    def close(self):
        """Close I2C bus"""
        self.bus.close()

    # ============================================
    # Pressure Conversion Functions
    # ============================================

    def voltage_to_relpressure(self, channel):
        """
        Converts voltage to relative pressure
        
        """
        voltage = self.read_voltage(channel)
        kpa = ((0.75 * (voltage - 0.5) - 1) * 100)
        
        # For positive pressure, return as is
        return kpa 
    
    def read_all_pressures(self):
        """
        Read pressures from all 4 channels
        
        Returns:
            List of pressures in kPa for channels 0-3
        """
        pressures = []
        for ch in range(4):
            pressure = self.voltage_to_relpressure(ch)
            pressures.append(pressure)
        return pressures
    
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
        adc = Transducer(bus_num=ADS1115_BUS, 
                      address=ADS1115_ADDRESS,
                      gain=CONFIG_PGA_4_096V)
    except Exception as e:
        print(f"Error initializing ADS1115: {e}")
        print("\nTroubleshooting:")
        print("- Ensure the ADS1115 is connected to the correct I2C bus.")
        print("- Verify the device address matches the hardware setup.")
        print("- Check wiring and power to the ADS1115 module.")
        return
    try:
        while True:
            pressures = adc.read_all_pressures()
            print(f"Pressures (kPa): A0={pressures[0]:6.2f}, A1={pressures[1]:6.2f}, A2={pressures[2]:6.2f}, A3={pressures[3]:6.2f}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n✓ Exiting pressure read loop")
    finally:
        adc.close() 

if __name__ == '__main__':
    main()