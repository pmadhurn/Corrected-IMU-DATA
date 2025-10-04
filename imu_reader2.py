#!/usr/bin/env python3
import serial
import struct
import datetime
import configparser
import threading
import time
from collections import deque

class FlexibleIMUReader:
    def __init__(self, config_file='config.ini'):
        # Read configuration
        self.config = configparser.ConfigParser()
        self.config.read(config_file)
        
        # Serial configuration
        self.port = self.config.get('Serial', 'port')
        self.baudrate = self.config.getint('Serial', 'baudrate')
        
        # Logging configuration
        self.log_file = self.config.get('Logging', 'log_file')
        self.max_entries = self.config.getint('Logging', 'max_entries')
        
        # Data buffer
        self.data_buffer = deque(maxlen=self.max_entries)
        
        # Frame parameters
        self.SYNC_BYTE = 0xFF
        self.START_BYTE = 0x02
        self.END_BYTE = 0x03
        self.CMD_CONTINUOUS = 0x90
        
        # Serial connection
        self.serial_port = None
        self.running = False
        self.debug = False
        
        # Auto-detected frame structure
        self.frame_length = None
        self.structure_detected = False
    
    def connect(self):
        """Establish serial connection"""
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            self.serial_port.reset_input_buffer()
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Disconnected")
    
    def find_frame_start(self):
        """Find the start of a valid frame"""
        sync_search = bytearray()
        
        while self.running:
            byte = self.serial_port.read(1)
            if not byte:
                return False
            
            sync_search.append(byte[0])
            if len(sync_search) > 2:
                sync_search.pop(0)
            
            if len(sync_search) == 2 and sync_search[0] == self.SYNC_BYTE and sync_search[1] == self.START_BYTE:
                return True
        
        return False
    
    def read_frame(self):
        """Read a complete frame from the serial port"""
        if not self.find_frame_start():
            return None
        
        # Read CMD byte
        cmd = self.serial_port.read(1)
        if not cmd or cmd[0] != self.CMD_CONTINUOUS:
            return None
        
        # Read length (2 bytes, big endian)
        len_bytes = self.serial_port.read(2)
        if len(len_bytes) != 2:
            return None
        
        data_length = struct.unpack('>H', len_bytes)[0]
        
        # Auto-detect frame length on first read
        if not self.structure_detected:
            self.frame_length = data_length
            self.detect_structure(data_length)
        
        # Read data
        data = self.serial_port.read(data_length)
        if len(data) != data_length:
            return None
        
        # Read CRC (2 bytes) - we'll skip verification
        crc_bytes = self.serial_port.read(2)
        if len(crc_bytes) != 2:
            return None
        
        # Read end byte
        end_byte = self.serial_port.read(1)
        if not end_byte or end_byte[0] != self.END_BYTE:
            return None
        
        return data
    
    def detect_structure(self, data_length):
        """Auto-detect which fields are present based on data length"""
        self.structure_detected = True
        
        if data_length == 106:
            print("✓ Detected 106-byte structure (without Temperatures and UTC)")
            self.has_temperatures = False
            self.has_utc = False
        elif data_length == 114:
            print("✓ Detected 114-byte structure (with Temperatures, without UTC)")
            self.has_temperatures = True
            self.has_utc = False
        elif data_length == 121:
            print("✓ Detected 121-byte structure (full, with Temperatures and UTC)")
            self.has_temperatures = True
            self.has_utc = True
        else:
            print(f"⚠ Unknown structure length: {data_length} bytes")
            # Default to minimal structure
            self.has_temperatures = False
            self.has_utc = False
    
    def parse_data(self, data):
        """Parse data based on detected structure"""
        parsed = {}
        offset = 0
        
        try:
            # Euler angles (12 bytes) - Always present
            parsed['roll'] = struct.unpack('<f', data[offset:offset+4])[0]
            parsed['pitch'] = struct.unpack('<f', data[offset+4:offset+8])[0]
            parsed['yaw'] = struct.unpack('<f', data[offset+8:offset+12])[0]
            offset += 12
            
            # Gyroscope (12 bytes) - Always present
            parsed['gx'] = struct.unpack('<f', data[offset:offset+4])[0]
            parsed['gy'] = struct.unpack('<f', data[offset+4:offset+8])[0]
            parsed['gz'] = struct.unpack('<f', data[offset+8:offset+12])[0]
            offset += 12
            
            # Accelerometer (12 bytes) - Always present
            parsed['ax'] = struct.unpack('<f', data[offset:offset+4])[0]
            parsed['ay'] = struct.unpack('<f', data[offset+4:offset+8])[0]
            parsed['az'] = struct.unpack('<f', data[offset+8:offset+12])[0]
            offset += 12
            
            # Magnetometer (12 bytes) - Always present
            parsed['mx'] = struct.unpack('<f', data[offset:offset+4])[0]
            parsed['my'] = struct.unpack('<f', data[offset+4:offset+8])[0]
            parsed['mz'] = struct.unpack('<f', data[offset+8:offset+12])[0]
            offset += 12
            
            # Temperatures (8 bytes) - Conditional
            if self.has_temperatures:
                parsed['t0'] = struct.unpack('<f', data[offset:offset+4])[0]
                parsed['t1'] = struct.unpack('<f', data[offset+4:offset+8])[0]
                offset += 8
            
            # Time (4 bytes)
            parsed['time'] = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            
            # iTOW (4 bytes)
            parsed['iTOW'] = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            
            # GPS flags + numSV (2 bytes)
            parsed['gps_flags'] = data[offset]
            parsed['num_sv'] = data[offset+1]
            gps_fix_type = parsed['gps_flags'] & 0x03
            parsed['gps_fix_string'] = ["NO_FIX", "TIME_ONLY", "2D_FIX", "3D_FIX"][gps_fix_type]
            offset += 2
            
            # BARO_Alt (4 bytes) - As signed int32, in centimeters
            baro_raw = struct.unpack('<i', data[offset:offset+4])[0]
            parsed['baro_alt'] = baro_raw / 100.0  # Convert to meters
            offset += 4
            
            # BARO_P (4 bytes)
            parsed['baro_p'] = struct.unpack('<f', data[offset:offset+4])[0]
            offset += 4
            
            # Position (24 bytes)
            parsed['lat'] = struct.unpack('<d', data[offset:offset+8])[0]
            parsed['long'] = struct.unpack('<d', data[offset+8:offset+16])[0]
            parsed['alt'] = struct.unpack('<d', data[offset+16:offset+24])[0]
            offset += 24
            
            # Velocity (12 bytes)
            parsed['vel_n'] = struct.unpack('<f', data[offset:offset+4])[0]
            parsed['vel_e'] = struct.unpack('<f', data[offset+4:offset+8])[0]
            parsed['vel_d'] = struct.unpack('<f', data[offset+8:offset+12])[0]
            offset += 12
            
            # UTC (7 bytes) - Conditional
            if self.has_utc:
                parsed['utc_year'] = struct.unpack('<H', data[offset:offset+2])[0]
                parsed['utc_month'] = data[offset+2]
                parsed['utc_day'] = data[offset+3]
                parsed['utc_hour'] = data[offset+4]
                parsed['utc_minute'] = data[offset+5]
                parsed['utc_second'] = data[offset+6]
                offset += 7
            
            # Heading (4 bytes)
            if offset + 4 <= len(data):
                parsed['heading'] = struct.unpack('<f', data[offset:offset+4])[0]
                offset += 4
            
            if self.debug:
                print(f"Parsed {offset}/{len(data)} bytes successfully")
                
        except Exception as e:
            if self.debug:
                print(f"Error parsing data: {e}")
            return None
        
        return parsed
    
    def format_log_entry(self, parsed_data):
        """Format parsed data into a log entry"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        parts = [timestamp]
        
        # Euler angles
        if all(k in parsed_data for k in ['roll', 'pitch', 'yaw']):
            roll_deg = parsed_data['roll'] * 57.2957795
            pitch_deg = parsed_data['pitch'] * 57.2957795
            yaw_deg = parsed_data['yaw'] * 57.2957795
            parts.append(f"EULER(deg): R={roll_deg:.2f},P={pitch_deg:.2f},Y={yaw_deg:.2f}")
        
        # Gyroscope
        if all(k in parsed_data for k in ['gx', 'gy', 'gz']):
            parts.append(f"GYRO: X={parsed_data['gx']:.3f},Y={parsed_data['gy']:.3f},Z={parsed_data['gz']:.3f}")
        
        # Accelerometer
        if all(k in parsed_data for k in ['ax', 'ay', 'az']):
            parts.append(f"ACCEL: X={parsed_data['ax']:.3f},Y={parsed_data['ay']:.3f},Z={parsed_data['az']:.3f}")
        
        # Magnetometer
        if all(k in parsed_data for k in ['mx', 'my', 'mz']):
            parts.append(f"MAG: X={parsed_data['mx']:.3f},Y={parsed_data['my']:.3f},Z={parsed_data['mz']:.3f}")
        
        # Temperatures (if present)
        if 't0' in parsed_data and 't1' in parsed_data:
            parts.append(f"TEMP: T0={parsed_data['t0']:.2f}°C,T1={parsed_data['t1']:.2f}°C")
        
        # Barometric altitude (FIXED!)
        if 'baro_alt' in parsed_data:
            parts.append(f"BARO_ALT: {parsed_data['baro_alt']:.2f}m")
        
        # GPS info
        if 'gps_fix_string' in parsed_data:
            parts.append(f"GPS: {parsed_data['gps_fix_string']},SAT={parsed_data.get('num_sv', 0)}")
            if 'iTOW' in parsed_data:
                parts.append(f"iTOW={parsed_data['iTOW']}ms")
        
        # Position
        if all(k in parsed_data for k in ['lat', 'long']):
            parts.append(f"POS: LAT={parsed_data['lat']:.6f},LON={parsed_data['long']:.6f}")
            if 'alt' in parsed_data:
                parts.append(f"ALT={parsed_data['alt']:.2f}m")
        
        # Velocity
        if all(k in parsed_data for k in ['vel_n', 'vel_e', 'vel_d']):
            parts.append(f"VEL: N={parsed_data['vel_n']:.2f},E={parsed_data['vel_e']:.2f},D={parsed_data['vel_d']:.2f}")
        
        # Heading
        if 'heading' in parsed_data:
            parts.append(f"HEADING: {parsed_data['heading']:.2f}°")
        
        # UTC (if present)
        if all(k in parsed_data for k in ['utc_year', 'utc_month', 'utc_day']):
            parts.append(f"UTC: {parsed_data['utc_year']}-{parsed_data['utc_month']:02d}-{parsed_data['utc_day']:02d} "
                        f"{parsed_data.get('utc_hour', 0):02d}:{parsed_data.get('utc_minute', 0):02d}:{parsed_data.get('utc_second', 0):02d}")
        
        return ",".join(parts)
    
    def write_log(self):
        """Write buffer to log file"""
        with open(self.log_file, 'w') as f:
            for entry in self.data_buffer:
                f.write(entry + '\n')
    
    def reading_thread(self):
        """Main reading thread"""
        frame_count = 0
        error_count = 0
        
        while self.running:
            try:
                frame_data = self.read_frame()
                if frame_data:
                    parsed_data = self.parse_data(frame_data)
                    if parsed_data:
                        log_entry = self.format_log_entry(parsed_data)
                        self.data_buffer.append(log_entry)
                        frame_count += 1
                        
                        # Print status
                        print(f"\rFrames: {frame_count}, Errors: {error_count} | {log_entry[:100]}...", end='', flush=True)
                        
                        # Write to file periodically
                        if len(self.data_buffer) % 100 == 0:
                            self.write_log()
                    else:
                        error_count += 1
                else:
                    error_count += 1
                
            except Exception as e:
                error_count += 1
                if self.debug:
                    print(f"\nError in reading thread: {e}")
                time.sleep(0.01)
    
    def start(self):
        """Start reading from IMU"""
        if not self.connect():
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self.reading_thread)
        self.thread.start()
        
        print("IMU reader started. Press Ctrl+C to stop.")
        return True
    
    def stop(self):
        """Stop reading and save final log"""
        print("\nStopping IMU reader...")
        self.running = False
        
        if hasattr(self, 'thread'):
            self.thread.join()
        
        self.write_log()
        self.disconnect()
        print(f"Data saved to {self.log_file}")

def main():
    reader = FlexibleIMUReader('config.ini')
    
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == '--debug':
        reader.debug = True
        print("Starting in debug mode")
    
    try:
        if reader.start():
            while True:
                time.sleep(0.1)
    except KeyboardInterrupt:
        reader.stop()

if __name__ == "__main__":
    main()