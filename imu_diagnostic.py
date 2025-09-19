#!/usr/bin/env python3
import serial
import struct
import time
import binascii

class IMUDiagnostic:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        
    def connect(self):
        """Connect to IMU"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=2.0
            )
            self.serial_port.reset_input_buffer()
            print(f"Connected to {self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def find_sync_pattern(self, duration=5):
        """Find and analyze sync patterns"""
        print(f"\nSearching for sync patterns for {duration} seconds...")
        
        data_buffer = bytearray()
        start_time = time.time()
        
        while time.time() - start_time < duration:
            data = self.serial_port.read(1024)
            if data:
                data_buffer.extend(data)
        
        print(f"Collected {len(data_buffer)} bytes")
        
        # Look for FF 02 pattern
        sync_positions = []
        for i in range(len(data_buffer) - 1):
            if data_buffer[i] == 0xFF and data_buffer[i+1] == 0x02:
                sync_positions.append(i)
        
        print(f"Found {len(sync_positions)} sync patterns")
        
        if len(sync_positions) >= 2:
            # Calculate frame lengths
            frame_lengths = []
            for i in range(len(sync_positions) - 1):
                frame_lengths.append(sync_positions[i+1] - sync_positions[i])
            
            avg_frame_length = sum(frame_lengths) / len(frame_lengths)
            print(f"Average frame length: {avg_frame_length:.1f} bytes")
            
            # Analyze first complete frame
            if sync_positions[0] + 130 < len(data_buffer):
                self.analyze_frame(data_buffer, sync_positions[0])
        
        return data_buffer, sync_positions
    
    def analyze_frame(self, data_buffer, start_pos):
        """Analyze a single frame structure"""
        print("\nAnalyzing frame structure:")
        
        # Extract frame components
        sync = data_buffer[start_pos]
        start = data_buffer[start_pos + 1]
        cmd = data_buffer[start_pos + 2]
        len_msb = data_buffer[start_pos + 3]
        len_lsb = data_buffer[start_pos + 4]
        
        data_length = (len_msb << 8) | len_lsb
        
        print(f"  Sync: 0x{sync:02X}")
        print(f"  Start: 0x{start:02X}")
        print(f"  CMD: 0x{cmd:02X}")
        print(f"  Length: {data_length} (0x{len_msb:02X} 0x{len_lsb:02X})")
        
        if start_pos + 5 + data_length + 3 <= len(data_buffer):
            data = data_buffer[start_pos + 5:start_pos + 5 + data_length]
            crc_bytes = data_buffer[start_pos + 5 + data_length:start_pos + 5 + data_length + 2]
            end_byte = data_buffer[start_pos + 5 + data_length + 2]
            
            received_crc = (crc_bytes[0] << 8) | crc_bytes[1]
            
            print(f"  Data: {data_length} bytes")
            print(f"  CRC: 0x{received_crc:04X}")
            print(f"  End: 0x{end_byte:02X}")
            
            # Test different CRC calculations
            self.test_crc_methods(cmd, len_msb, len_lsb, data, received_crc)
            
            # Show first few data values
            print("\nFirst few data values:")
            for i in range(min(12, len(data))):
                if i % 4 == 0:
                    try:
                        float_val = struct.unpack('<f', data[i:i+4])[0]
                        print(f"  Offset {i:3d}: {float_val:10.6f} (float)")
                    except:
                        pass
    
    def test_crc_methods(self, cmd, len_msb, len_lsb, data, received_crc):
        """Test various CRC calculation methods"""
        print("\nTesting CRC methods:")
        
        # Different CRC polynomials and methods
        def crc16_modbus(data):
            crc = 0xFFFF
            for byte in data:
                crc ^= byte
                for _ in range(8):
                    if crc & 0x0001:
                        crc = (crc >> 1) ^ 0xA001
                    else:
                        crc >>= 1
            return crc
        
        def crc16_xmodem(data):
            crc = 0x0000
            for byte in data:
                crc ^= byte << 8
                for _ in range(8):
                    if crc & 0x8000:
                        crc = (crc << 1) ^ 0x1021
                    else:
                        crc <<= 1
                    crc &= 0xFFFF
            return crc
        
        def crc16_ccitt(data):
            crc = 0xFFFF
            for byte in data:
                crc ^= byte << 8
                for _ in range(8):
                    if crc & 0x8000:
                        crc = (crc << 1) ^ 0x1021
                    else:
                        crc <<= 1
                    crc &= 0xFFFF
            return crc
        
        # Test different data combinations
        tests = [
            ("Data only", data),
            ("CMD + Data", bytes([cmd]) + data),
            ("CMD + Length + Data", bytes([cmd, len_msb, len_lsb]) + data),
            ("Length + Data", bytes([len_msb, len_lsb]) + data),
        ]
        
        for name, test_data in tests:
            crc_modbus = crc16_modbus(test_data)
            crc_xmodem = crc16_xmodem(test_data)
            crc_ccitt = crc16_ccitt(test_data)
            
            print(f"\n  {name}:")
            print(f"    MODBUS: 0x{crc_modbus:04X} {'✓' if crc_modbus == received_crc else '✗'}")
            print(f"    XMODEM: 0x{crc_xmodem:04X} {'✓' if crc_xmodem == received_crc else '✗'}")
            print(f"    CCITT:  0x{crc_ccitt:04X} {'✓' if crc_ccitt == received_crc else '✗'}")
    
    def capture_raw_frames(self, count=10):
        """Capture and save raw frames"""
        print(f"\nCapturing {count} raw frames...")
        
        frames = []
        frame_count = 0
        
        while frame_count < count:
            # Look for sync
            sync_found = False
            while not sync_found:
                b1 = self.serial_port.read(1)
                if b1 and b1[0] == 0xFF:
                    b2 = self.serial_port.read(1)
                    if b2 and b2[0] == 0x02:
                        sync_found = True
            
            # Read rest of frame
            frame = bytearray([0xFF, 0x02])
            
            # CMD
            cmd = self.serial_port.read(1)
            if not cmd:
                continue
            frame.extend(cmd)
            
            # Length
            length_bytes = self.serial_port.read(2)
            if len(length_bytes) != 2:
                continue
            frame.extend(length_bytes)
            
            data_length = struct.unpack('>H', length_bytes)[0]
            
            # Data + CRC + End
            remaining = self.serial_port.read(data_length + 3)
            if len(remaining) != data_length + 3:
                continue
            
            frame.extend(remaining)
            frames.append(frame)
            frame_count += 1
            print(f"  Frame {frame_count}: {len(frame)} bytes")
        
        # Save frames
        with open('raw_frames.bin', 'wb') as f:
            for frame in frames:
                f.write(frame)
        
        print(f"Saved {len(frames)} frames to raw_frames.bin")
        
        # Analyze frames
        print("\nFrame analysis:")
        for i, frame in enumerate(frames):
            print(f"\nFrame {i+1}:")
            print(f"  Hex: {frame[:20].hex()}...")
            if len(frame) > 5:
                data_length = struct.unpack('>H', frame[3:5])[0]
                print(f"  Data length: {data_length}")
                if len(frame) >= data_length + 8:
                    crc_bytes = frame[5+data_length:7+data_length]
                    crc = struct.unpack('>H', crc_bytes)[0]
                    print(f"  CRC: 0x{crc:04X}")

def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 imu_diagnostic.py /dev/ttyUSB0")
        return
    
    port = sys.argv[1]
    diag = IMUDiagnostic(port)
    
    if not diag.connect():
        return
    
    try:
        # Run diagnostics
        diag.find_sync_pattern(duration=3)
        diag.capture_raw_frames(count=5)
        
    except KeyboardInterrupt:
        print("\nDiagnostic stopped")

    finally:
        if diag.serial_port:
            diag.serial_port.close()

if __name__ == "__main__":
    main()