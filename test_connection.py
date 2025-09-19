#!/usr/bin/env python3
import serial
import time
import sys

def test_connection(port, baudrate=115200):
    print(f"Testing {port} at {baudrate} baud...")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print("Port opened successfully")
        
        print("\nReading raw data for 3 seconds...")
        start = time.time()
        byte_count = 0
        ff_count = 0
        
        while time.time() - start < 3:
            data = ser.read(100)
            byte_count += len(data)
            ff_count += data.count(0xFF)
            
            # Show sample
            if data and byte_count < 200:
                print(f"Sample: {' '.join(f'{b:02X}' for b in data[:20])}")
        
        print(f"\nReceived {byte_count} bytes in 3 seconds")
        print(f"Data rate: {byte_count/3:.0f} bytes/second")
        print(f"Found {ff_count} sync bytes (0xFF)")
        
        if byte_count > 0:
            print("\n✓ IMU appears to be transmitting data")
            expected_rate = 100 * 130  # 100Hz * ~130 bytes/frame
            if byte_count > expected_rate * 0.8:
                print("✓ Data rate looks correct for 100Hz operation")
        else:
            print("\n✗ No data received - check connection and power")
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 test_connection.py /dev/ttyUSB0")
        sys.exit(1)
    
    test_connection(sys.argv[1])