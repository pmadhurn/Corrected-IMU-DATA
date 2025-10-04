<<<<<<< HEAD
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
        print("\n" + "="*60)
        print("FRAME STRUCTURE ANALYSIS")
        print("="*60)
        
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
        print(f"  Length: {data_length} bytes (0x{len_msb:02X} 0x{len_lsb:02X})")
        
        if start_pos + 5 + data_length + 3 <= len(data_buffer):
            data = data_buffer[start_pos + 5:start_pos + 5 + data_length]
            crc_bytes = data_buffer[start_pos + 5 + data_length:start_pos + 5 + data_length + 2]
            end_byte = data_buffer[start_pos + 5 + data_length + 2]
            
            received_crc = (crc_bytes[0] << 8) | crc_bytes[1]
            
            print(f"  Data: {data_length} bytes")
            print(f"  CRC: 0x{received_crc:04X}")
            print(f"  End: 0x{end_byte:02X}")
            
            # Show complete hex dump
            print(f"\n  Complete frame hex ({len(data)} bytes):")
            for i in range(0, len(data), 16):
                hex_str = ' '.join(f'{b:02x}' for b in data[i:i+16])
                print(f"    {i:04d}: {hex_str}")
            
            # Detailed field analysis
            self.analyze_all_fields(data)
            
            # Test different CRC calculations
            self.test_crc_methods(cmd, len_msb, len_lsb, data, received_crc)
    
    def analyze_all_fields(self, data):
        """Analyze all expected fields"""
        print("\n" + "="*60)
        print("FIELD-BY-FIELD ANALYSIS")
        print("="*60)
        
        # Based on 106 bytes, some fields are missing
        # Let's parse what we CAN identify
        
        offset = 0
        
        # Euler angles (12 bytes) - Usually always present
        if offset + 12 <= len(data):
            roll = struct.unpack('<f', data[offset:offset+4])[0]
            pitch = struct.unpack('<f', data[offset+4:offset+8])[0]
            yaw = struct.unpack('<f', data[offset+8:offset+12])[0]
            print(f"\n[{offset:3d}-{offset+11:3d}] EULER ANGLES (12 bytes):")
            print(f"  Roll:  {roll:.6f} rad = {roll*180/3.14159:.2f}°")
            print(f"  Pitch: {pitch:.6f} rad = {pitch*180/3.14159:.2f}°")
            print(f"  Yaw:   {yaw:.6f} rad = {yaw*180/3.14159:.2f}°")
            offset += 12
        
        # Gyroscope (12 bytes)
        if offset + 12 <= len(data):
            gx = struct.unpack('<f', data[offset:offset+4])[0]
            gy = struct.unpack('<f', data[offset+4:offset+8])[0]
            gz = struct.unpack('<f', data[offset+8:offset+12])[0]
            print(f"\n[{offset:3d}-{offset+11:3d}] GYROSCOPE (12 bytes):")
            print(f"  Gx: {gx:.6f} rad/s")
            print(f"  Gy: {gy:.6f} rad/s")
            print(f"  Gz: {gz:.6f} rad/s")
            offset += 12
        
        # Accelerometer (12 bytes)
        if offset + 12 <= len(data):
            ax = struct.unpack('<f', data[offset:offset+4])[0]
            ay = struct.unpack('<f', data[offset+4:offset+8])[0]
            az = struct.unpack('<f', data[offset+8:offset+12])[0]
            print(f"\n[{offset:3d}-{offset+11:3d}] ACCELEROMETER (12 bytes):")
            print(f"  Ax: {ax:.6f} m/s²")
            print(f"  Ay: {ay:.6f} m/s²")
            print(f"  Az: {az:.6f} m/s²")
            offset += 12
        
        # Magnetometer (12 bytes)
        if offset + 12 <= len(data):
            mx = struct.unpack('<f', data[offset:offset+4])[0]
            my = struct.unpack('<f', data[offset+4:offset+8])[0]
            mz = struct.unpack('<f', data[offset+8:offset+12])[0]
            print(f"\n[{offset:3d}-{offset+11:3d}] MAGNETOMETER (12 bytes):")
            print(f"  Mx: {mx:.6f} µT")
            print(f"  My: {my:.6f} µT")
            print(f"  Mz: {mz:.6f} µT")
            offset += 12
        
        # Now we're at byte 48
        # Check if temperatures are present (8 bytes) or skipped
        print(f"\n[{offset:3d}-???] REMAINING {len(data)-offset} BYTES")
        print("Testing different field combinations...\n")
        
        # Test scenario 1: Temperatures present (T0, T1 = 8 bytes)
        print("SCENARIO 1: With Temperatures (T0, T1)")
        test_offset = offset
        if test_offset + 8 <= len(data):
            t0 = struct.unpack('<f', data[test_offset:test_offset+4])[0]
            t1 = struct.unpack('<f', data[test_offset+4:test_offset+8])[0]
            print(f"  [{test_offset:3d}-{test_offset+7:3d}] T0: {t0:.2f}°C, T1: {t1:.2f}°C")
            test_offset += 8
            self.test_remaining_fields(data, test_offset, "with temps")
        
        # Test scenario 2: Skip temperatures
        print("\nSCENARIO 2: Without Temperatures")
        test_offset = offset
        self.test_remaining_fields(data, test_offset, "without temps")
    
    def test_remaining_fields(self, data, offset, scenario):
        """Test parsing remaining fields"""
        
        # Time (4 bytes)
        if offset + 4 <= len(data):
            time_val = struct.unpack('<I', data[offset:offset+4])[0]
            print(f"  [{offset:3d}-{offset+3:3d}] Time: {time_val}")
            offset += 4
        
        # iTOW (4 bytes)
        if offset + 4 <= len(data):
            itow = struct.unpack('<I', data[offset:offset+4])[0]
            print(f"  [{offset:3d}-{offset+3:3d}] iTOW: {itow} ms")
            offset += 4
        
        # GPS Flags + numSV (2 bytes)
        if offset + 2 <= len(data):
            gps_flags = data[offset]
            num_sv = data[offset+1]
            fix_type = gps_flags & 0x03
            fix_str = ["NO_FIX", "TIME_ONLY", "2D_FIX", "3D_FIX"][fix_type]
            print(f"  [{offset:3d}-{offset+1:3d}] GPS: {fix_str}, Satellites: {num_sv}")
            offset += 2
        
        # BARO_Alt (4 bytes) - THE PROBLEMATIC FIELD
        if offset + 4 <= len(data):
            raw_bytes = data[offset:offset+4]
            baro_alt_int = struct.unpack('<i', raw_bytes)[0]
            baro_alt_float = struct.unpack('<f', raw_bytes)[0]
            print(f"  [{offset:3d}-{offset+3:3d}] BARO_Alt (hex: {raw_bytes.hex()}):")
            print(f"    As int32: {baro_alt_int} -> {baro_alt_int/100.0:.2f}m")
            print(f"    As float: {baro_alt_float:.2f}m")
            
            # Highlight reasonable values
            if -100 < baro_alt_int/100.0 < 10000:
                print(f"    ✓ REASONABLE as int32/100")
            if -100 < baro_alt_float < 10000:
                print(f"    ✓ REASONABLE as float")
            
            offset += 4
        
        # BARO_P (4 bytes)
        if offset + 4 <= len(data):
            baro_p = struct.unpack('<f', data[offset:offset+4])[0]
            print(f"  [{offset:3d}-{offset+3:3d}] BARO_P: {baro_p:.2f} Pa")
            offset += 4
        
        # Position (24 bytes: Lat, Long, Alt)
        if offset + 24 <= len(data):
            lat = struct.unpack('<d', data[offset:offset+8])[0]
            lon = struct.unpack('<d', data[offset+8:offset+16])[0]
            alt = struct.unpack('<d', data[offset+16:offset+24])[0]
            print(f"  [{offset:3d}-{offset+23:3d}] Position:")
            print(f"    Lat: {lat:.6f}°")
            print(f"    Lon: {lon:.6f}°")
            print(f"    Alt: {alt:.2f}m")
            offset += 24
        
        # Velocity (12 bytes)
        if offset + 12 <= len(data):
            vel_n = struct.unpack('<f', data[offset:offset+4])[0]
            vel_e = struct.unpack('<f', data[offset+4:offset+8])[0]
            vel_d = struct.unpack('<f', data[offset+8:offset+12])[0]
            print(f"  [{offset:3d}-{offset+11:3d}] Velocity: N={vel_n:.2f}, E={vel_e:.2f}, D={vel_d:.2f} m/s")
            offset += 12
        
        # UTC (7 bytes) - if present
        if offset + 7 <= len(data):
            utc_year = struct.unpack('<H', data[offset:offset+2])[0]
            utc_month = data[offset+2]
            utc_day = data[offset+3]
            utc_hour = data[offset+4]
            utc_min = data[offset+5]
            utc_sec = data[offset+6]
            print(f"  [{offset:3d}-{offset+6:3d}] UTC: {utc_year}-{utc_month:02d}-{utc_day:02d} {utc_hour:02d}:{utc_min:02d}:{utc_sec:02d}")
            offset += 7
        
        # Heading (4 bytes)
        if offset + 4 <= len(data):
            heading = struct.unpack('<f', data[offset:offset+4])[0]
            print(f"  [{offset:3d}-{offset+3:3d}] Heading: {heading:.2f}°")
            offset += 4
        
        # Summary
        bytes_used = offset
        bytes_remaining = len(data) - offset
        print(f"\n  Scenario '{scenario}': Used {bytes_used}/{len(data)} bytes, {bytes_remaining} remaining")
        
        if bytes_remaining == 0:
            print(f"  ✓✓✓ PERFECT MATCH! All bytes accounted for.")
        
    def test_crc_methods(self, cmd, len_msb, len_lsb, data, received_crc):
        """Test various CRC calculation methods"""
        print("\n" + "="*60)
        print("CRC VERIFICATION")
        print("="*60)
        
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
        
        print(f"Received CRC: 0x{received_crc:04X}\n")
        
        for name, test_data in tests:
            crc_modbus = crc16_modbus(test_data)
            crc_xmodem = crc16_xmodem(test_data)
            crc_ccitt = crc16_ccitt(test_data)
            
            print(f"  {name}:")
            match_modbus = "✓" if crc_modbus == received_crc else "✗"
            match_xmodem = "✓" if crc_xmodem == received_crc else "✗"
            match_ccitt = "✓" if crc_ccitt == received_crc else "✗"
            
            print(f"    MODBUS: 0x{crc_modbus:04X} {match_modbus}")
            print(f"    XMODEM: 0x{crc_xmodem:04X} {match_xmodem}")
            print(f"    CCITT:  0x{crc_ccitt:04X} {match_ccitt}")
    
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
=======
>>>>>>> e9a4d933daed8e167a3e3d0e1222691916cfd146
