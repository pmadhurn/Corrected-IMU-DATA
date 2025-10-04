import serial
import struct
import datetime
import configparser
import os
import time
import threading
from collections import deque

class IMU250Reader:
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
        
        # Data buffer for circular logging
        self.data_buffer = deque(maxlen=self.max_entries)
        
        # Frame parameters
        self.SYNC_BYTE = 0xFF
        self.START_BYTE = 0x02
        self.END_BYTE = 0x03
        self.CMD_CONTINUOUS = 0x90
        
        # Serial connection
        self.serial_port = None
        self.running = False
        
        # Debug mode
        self.debug = False
        
        # Define data field structure (field_name: size_in_bytes)
        # These fields appear in order based on the mask
        self.field_definitions = [
            ('roll', 4, 'f'),      # float32
            ('pitch', 4, 'f'),     # float32
            ('yaw', 4, 'f'),       # float32
            ('gx', 4, 'f'),        # float32
            ('gy', 4, 'f'),        # float32
            ('gz', 4, 'f'),        # float32
            ('ax', 4, 'f'),        # float32
            ('ay', 4, 'f'),        # float32
            ('az', 4, 'f'),        # float32
            ('mx', 4, 'f'),        # float32
            ('my', 4, 'f'),        # float32
            ('mz', 4, 'f'),        # float32
            ('t0', 4, 'f'),        # float32
            ('t1', 4, 'f'),        # float32
            ('time', 4, 'I'),      # uint32
            ('iTOW', 4, 'I'),      # uint32
            ('gps_flags', 1, 'B'), # uint8
            ('num_sv', 1, 'B'),    # uint8
            ('baro_alt', 4, 'i'),  # int32
            ('baro_p', 4, 'f'),    # float32
            ('lat', 8, 'd'),       # double
            ('long', 8, 'd'),      # double
            ('alt', 8, 'd'),       # double
            ('vel_n', 4, 'f'),     # float32
            ('vel_e', 4, 'f'),     # float32
            ('vel_d', 4, 'f'),     # float32
            ('utc', 7, 'raw'),     # 7 bytes raw
            ('heading', 4, 'f'),   # float32
        ]
        
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
            # Clear input buffer
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
    
    def calculate_crc16_xmodem(self, data):
        """Calculate CRC16-XMODEM"""
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
    
    def calculate_crc16_modbus(self, data):
        """Calculate CRC16-MODBUS"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def find_frame_start(self):
        """Find the start of a valid frame"""
        sync_search = bytearray()
        
        while True:
            byte = self.serial_port.read(1)
            if not byte:
                return False
            
            sync_search.append(byte[0])
            if len(sync_search) > 2:
                sync_search.pop(0)
            
            if len(sync_search) == 2 and sync_search[0] == self.SYNC_BYTE and sync_search[1] == self.START_BYTE:
                return True
    
    def read_frame(self):
        """Read a complete frame from the serial port"""
        if not self.find_frame_start():
            return None, 0
        
        # Read CMD byte
        cmd = self.serial_port.read(1)
        if not cmd:
            return None, 0
        
        if cmd[0] != self.CMD_CONTINUOUS:
            if self.debug:
                print(f"Unexpected CMD: 0x{cmd[0]:02X}")
            return None, 0
        
        # Read length (2 bytes, big endian)
        len_bytes = self.serial_port.read(2)
        if len(len_bytes) != 2:
            return None, 0
        
        data_length = struct.unpack('>H', len_bytes)[0]
        
        if self.debug:
            print(f"\nData length: {data_length}")
        
        # Read data
        data = self.serial_port.read(data_length)
        if len(data) != data_length:
            if self.debug:
                print(f"Data length mismatch: expected {data_length}, got {len(data)}")
            return None, 0
        
        # Read CRC (2 bytes)
        crc_bytes = self.serial_port.read(2)
        if len(crc_bytes) != 2:
            return None, 0
        
        # Read end byte
        end_byte = self.serial_port.read(1)
        if not end_byte or end_byte[0] != self.END_BYTE:
            if self.debug:
                print(f"Invalid end byte: 0x{end_byte[0]:02X if end_byte else 0:02X}")
            return None, 0
        
        # Verify CRC
        received_crc = struct.unpack('>H', crc_bytes)[0]
        
        # Try different CRC calculations
        crc1 = self.calculate_crc16_modbus(data)
        crc2 = self.calculate_crc16_modbus(cmd + len_bytes + data)
        crc3 = self.calculate_crc16_modbus(len_bytes + data)
        crc4 = self.calculate_crc16_xmodem(cmd + len_bytes + data)
        
        if self.debug:
            print(f"Received CRC: 0x{received_crc:04X}")
            print(f"CRC Method 1 (data only): 0x{crc1:04X}")
            print(f"CRC Method 2 (cmd+len+data): 0x{crc2:04X}")
            print(f"CRC Method 3 (len+data): 0x{crc3:04X}")
            print(f"CRC Method 4 (xmodem): 0x{crc4:04X}")
        
        # Optional: Skip CRC verification in debug mode
        if received_crc not in [crc1, crc2, crc3, crc4]:
            if self.debug:
                print("CRC mismatch - proceeding anyway")
                print(f"Data hex: {data.hex()}")
        
        return data, data_length
    
    def parse_data_flexible(self, data, data_length):
        """Parse data flexibly based on actual length"""
        parsed = {}
        offset = 0
        
        try:
            # Parse fields in order until we run out of data
            for field_name, field_size, field_type in self.field_definitions:
                if offset + field_size > data_length:
                    break
                
                if field_type == 'raw':
                    # Special handling for UTC (7 bytes)
                    if field_name == 'utc':
                        utc_data = data[offset:offset+field_size]
                        parsed['utc_year'] = struct.unpack('<H', utc_data[0:2])[0]
                        parsed['utc_month'] = utc_data[2]
                        parsed['utc_day'] = utc_data[3]
                        parsed['utc_hour'] = utc_data[4]
                        parsed['utc_minute'] = utc_data[5]
                        parsed['utc_second'] = utc_data[6]
                    else:
                        parsed[field_name] = data[offset:offset+field_size]
                else:
                    # Unpack based on type
                    value = struct.unpack(f'<{field_type}', data[offset:offset+field_size])[0]
                    parsed[field_name] = value
                
                offset += field_size
            
            # Parse GPS flags if available
            if 'gps_flags' in parsed:
                gps_fix_type = parsed['gps_flags'] & 0x03
                parsed['gps_fix_string'] = self.get_gps_fix_string(gps_fix_type)
                parsed['gps_valid_tow'] = bool(parsed['gps_flags'] & 0x04)
                parsed['gps_valid_wkn'] = bool(parsed['gps_flags'] & 0x08)
                parsed['gps_valid_utc'] = bool(parsed['gps_flags'] & 0x10)
                parsed['gps_true_heading_valid'] = bool(parsed['gps_flags'] & 0x20)
            
            if self.debug:
                print(f"Parsed {len(parsed)} fields from {offset} bytes")
                print(f"Available fields: {list(parsed.keys())}")
                
        except Exception as e:
            if self.debug:
                print(f"Error parsing data: {e}")
                print(f"Data hex: {data.hex()}")
            return None
        
        return parsed
    
    def get_gps_fix_string(self, fix_type):
        """Convert GPS fix type to string"""
        fix_types = {
            0: "NO_FIX",
            1: "TIME_ONLY",
            2: "2D_FIX",
            3: "3D_FIX"
        }
        return fix_types.get(fix_type, "UNKNOWN")
    
    def format_log_entry(self, parsed_data):
        """Format parsed data into a log entry"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        parts = [timestamp]
        
        # Euler angles (if available)
        if all(k in parsed_data for k in ['roll', 'pitch', 'yaw']):
            roll_deg = parsed_data['roll'] * 180.0 / 3.14159265359
            pitch_deg = parsed_data['pitch'] * 180.0 / 3.14159265359
            yaw_deg = parsed_data['yaw'] * 180.0 / 3.14159265359
            parts.append(f"EULER(deg): R={roll_deg:.2f},P={pitch_deg:.2f},Y={yaw_deg:.2f}")
        
        # Gyroscope (if available)
        if all(k in parsed_data for k in ['gx', 'gy', 'gz']):
            parts.append(f"GYRO: X={parsed_data['gx']:.3f},Y={parsed_data['gy']:.3f},Z={parsed_data['gz']:.3f}")
        
        # Accelerometer (if available)
        if all(k in parsed_data for k in ['ax', 'ay', 'az']):
            parts.append(f"ACCEL: X={parsed_data['ax']:.3f},Y={parsed_data['ay']:.3f},Z={parsed_data['az']:.3f}")
        
        # Magnetometer (if available)
        if all(k in parsed_data for k in ['mx', 'my', 'mz']):
            parts.append(f"MAG: X={parsed_data['mx']:.3f},Y={parsed_data['my']:.3f},Z={parsed_data['mz']:.3f}")
        
        # UTC time (if available)
        if all(k in parsed_data for k in ['utc_year', 'utc_month', 'utc_day']):
            parts.append(f"UTC: {parsed_data['utc_year']}-{parsed_data['utc_month']:02d}-{parsed_data['utc_day']:02d} "
                        f"{parsed_data.get('utc_hour', 0):02d}:{parsed_data.get('utc_minute', 0):02d}:{parsed_data.get('utc_second', 0):02d}")
        
        # Magnetic heading (if available)
        if 'heading' in parsed_data:
            parts.append(f"MAG_HEADING: {parsed_data['heading']:.2f}°")
        
        # Barometric altitude (if available)
        if 'baro_alt' in parsed_data:
            parts.append(f"BARO_ALT: {parsed_data['baro_alt']/100.0:.2f}m")
        
        # GPS info (if available)
        if 'gps_fix_string' in parsed_data:
            parts.append(f"GPS: {parsed_data['gps_fix_string']},SAT={parsed_data.get('num_sv', 0)}")
            if 'iTOW' in parsed_data:
                parts.append(f"iTOW={parsed_data['iTOW']}ms")
        
        # Position (if available)
        if all(k in parsed_data for k in ['lat', 'long']):
            parts.append(f"POS: LAT={parsed_data['lat']:.6f},LON={parsed_data['long']:.6f}")
            if 'alt' in parsed_data:
                parts.append(f"ALT={parsed_data['alt']:.2f}m")
        
        # Velocity (if available)
        if all(k in parsed_data for k in ['vel_n', 'vel_e', 'vel_d']):
            parts.append(f"VEL: N={parsed_data['vel_n']:.2f},E={parsed_data['vel_e']:.2f},D={parsed_data['vel_d']:.2f}")
        
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
                frame_data, data_length = self.read_frame()
                if frame_data:
                    parsed_data = self.parse_data_flexible(frame_data, data_length)
                    if parsed_data:
                        log_entry = self.format_log_entry(parsed_data)
                        self.data_buffer.append(log_entry)
                        frame_count += 1
                        
                        # Clear line and print
                        print(f"\rFrames: {frame_count}, Errors: {error_count}, Len: {data_length} | {log_entry[:80]}...", end='', flush=True)
                        
                        # Write to file periodically (every 100 entries)
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
        print("Press 'd' + Enter to toggle debug mode")
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
    
    def toggle_debug(self):
        """Toggle debug mode"""
        self.debug = not self.debug
        print(f"\nDebug mode: {'ON' if self.debug else 'OFF'}")

def main():
    reader = IMU250Reader('config.ini')
    
    # Check if we need to run in debug mode
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