#!/usr/bin/env python3
import serial.tools.list_ports
import serial
import time
import struct

def list_serial_ports():
    """List all available serial ports"""
    print("Available serial ports:")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"  {port.device}: {port.description}")
    return [port.device for port in ports]

def test_imu_connection(port, baudrate=115200):
    """Test connection to IMU"""
    print(f"\nTesting connection to {port} at {baudrate} baud...")
    
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=2.0
        )
        
        print("Port opened successfully")
        print("Waiting for data...")
        
        # Look for sync bytes
        start_time = time.time()
        sync_found = False
        
        while time.time() - start_time < 5:  # 5 second timeout
            byte = ser.read(1)
            if byte and byte[0] == 0xFF:
                next_byte = ser.read(1)
                if next_byte and next_byte[0] == 0x02:
                    sync_found = True
                    print("✓ Found IMU data stream!")
                    break
        
        if not sync_found:
            print("✗ No IMU data detected")
        
        ser.close()
        return sync_found
        
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def raw_data_dump(port, baudrate=115200, duration=5):
    """Dump raw data for debugging"""
    print(f"\nDumping raw data from {port} for {duration} seconds...")
    
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=1.0)
        start_time = time.time()
        
        with open('raw_dump.bin', 'wb') as f:
            while time.time() - start_time < duration:
                data = ser.read(1024)
                if data:
                    f.write(data)
                    print(f"Read {len(data)} bytes", end='\r')
        
        ser.close()
        print(f"\nRaw data saved to raw_dump.bin")
        
    except Exception as e:
        print(f"Error: {e}")

def main():
    print("IMU 560 Connection Tool")
    print("=" * 50)
    
    # List ports
    ports = list_serial_ports()
    
    if not ports:
        print("No serial ports found!")
        return
    
    # Test each port
    print("\nTesting ports for IMU...")
    valid_port = None
    
    for port in ports:
        if 'USB' in port or 'ACM' in port:  # Common USB serial port names
            if test_imu_connection(port):
                valid_port = port
                break
    
    if valid_port:
        print(f"\n✓ IMU detected on {valid_port}")
        print(f"\nUpdate your config.ini with:")
        print(f"port = {valid_port}")
        
        # Option to dump raw data
        response = input("\nDump raw data for analysis? (y/n): ")
        if response.lower() == 'y':
            raw_data_dump(valid_port)
    else:
        print("\n✗ No IMU detected on any port")
        print("Please check:")
        print("  1. IMU is powered on")
        print("  2. USB cable is connected")
        print("  3. Correct drivers are installed")

if __name__ == "__main__":
    main()