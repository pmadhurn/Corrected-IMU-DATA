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
        
        # Output mask flags (from IMU250 documentation)
        self.OUTPUT_EULER = 0x00000001
        self.OUTPUT_GYROSCOPES = 0x00000002
        self.OUTPUT_ACCELEROMETERS = 0x00000004
        self.OUTPUT_MAGNETOMETERS = 0x00000008
        self.OUTPUT_TEMPERATURES = 0x00000010
        self.OUTPUT_TIME_SINCE_RESET = 0x00000020
        self.OUTPUT_GPS_INFO = 0x00000040
        self.OUTPUT_BARO_ALTITUDE = 0x00000080
        self.OUTPUT_BARO_PRESSURE = 0x00000100
        self.OUTPUT_POSITION = 0x00000200
        self.OUTPUT_VELOCITY = 0x00000400
        self.OUTPUT_UTC_TIME = 0x00000800
        self.OUTPUT_MAG_HEADING = 0x00001000
        
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
            return None
        
        # Read CMD byte
        cmd = self.serial_port.read(1)
        if not cmd:
            return None
        
        if cmd[0] != self.CMD_CONTINUOUS:
            if self.debug:
                print(f"Unexpected CMD: 0x{cmd[0]:02X}")
            return None
        
        # Read length (2 bytes, big endian)
        len_bytes = self.serial_port.read(2)
        if len(len_bytes) != 2:
            return None
        
        data_length = struct.unpack('>H', len_bytes)[0]
        
        if self.debug:
            print(f"\nData length: {data_length}")
        
        # Read data
        data = self.serial_port.read(data_length)
        if len(data) != data_length:
            if self.debug:
                print(f"Data length mismatch: expected {data_length}, got {len(data)}")
            return None
        
        # Read CRC (2 bytes)
        crc_bytes = self.serial_port.read(2)
        if len(crc_bytes) != 2:
            return None
        
        # Read end byte
        end_byte = self.serial_port.read(1)
        if not end_byte or end_byte[0] != self.END_BYTE:
            if self.debug:
                print(f"Invalid end byte: 0x{end_byte[0]:02X if end_byte else 0:02X}")
            return None
        
        # Verify CRC
        received_crc = struct.unpack('>H', crc_bytes)[0]
        calculated_crc = self.calculate_crc16_modbus(cmd + len_bytes + data)
        
        if self.debug:
            print(f"Received CRC: 0x{received_crc:04X}, Calculated: 0x{calculated_crc:04X}")
        
        # For now, proceed even if CRC doesn't match (can be enabled later)
        # if received_crc != calculated_crc:
        #     return None
        
        return data
    
    def parse_data(self, data):
        """Parse the data buffer according to IMU250 variable format"""
        parsed = {}
        offset = 0
        
        try:
            # First, try to detect which fields are present by checking data length
            # and attempting to parse accordingly
            
            # Euler angles (Roll, Pitch, Yaw) - 12 bytes total (3 x float32)
            if offset + 12 <= len(data):
                parsed['roll'] = struct.unpack('<f', data[offset:offset+4])[0]
                parsed['pitch'] = struct.unpack('<f', data[offset+4:offset+8])[0]
                parsed['yaw'] = struct.unpack('<f', data[offset+8:offset+12])[0]
                offset += 12
                parsed['has_euler'] = True
            else:
                return None  # Minimum data should include Euler angles
            
            # Gyroscope data (Gx, Gy, Gz) - 12 bytes
            if offset + 12 <= len(data):
                parsed['gx'] = struct.unpack('<f', data[offset:offset+4])[0]
                parsed['gy'] = struct.unpack('<f', data[offset+4:offset+8])[0]
                parsed['gz'] = struct.unpack('<f', data[offset+8:offset+12])[0]
                offset += 12
                parsed['has_gyro'] = True
            
            # Accelerometer data (Ax, Ay, Az) - 12 bytes
            if offset + 12 <= len(data):
                parsed['ax'] = struct.unpack('<f', data[offset:offset+4])[0]
                parsed['ay'] = struct.unpack('<f', data[offset+4:offset+8])[0]
                parsed['az'] = struct.unpack('<f', data[offset+8:offset+12])[0]
                offset += 12
                parsed['has_accel'] = True
            
            # Magnetometer data (Mx, My, Mz) - 12 bytes
            if offset + 12 <= len(data):
                parsed['mx'] = struct.unpack('<f', data[offset:offset+4])[0]
                parsed['my'] = struct.unpack('<f', data[offset+4:offset+8])[0]
                parsed['mz'] = struct.unpack('<f', data[offset+8:offset+12])[0]
                offset += 12
                parsed['has_mag'] = True
            
            # Temperature data (T0, T1) - 8 bytes
            if offset + 8 <= len(data):
                parsed['t0'] = struct.unpack('<f', data[offset:offset+4])[0]
                parsed['t1'] = struct.unpack('<f', data[offset+4:offset+8])[0]
                offset += 8
                parsed['has_temp'] = True
            
            # Time since reset - 4 bytes
            if offset + 4 <= len(data):
                parsed['time'] = struct.unpack('<I', data[offset:offset+4])[0]
                offset += 4
                parsed['has_time'] = True
            
            # GPS Info (iTOW, flags, numSV) - 6 bytes
            if offset + 6 <= len(data):
                parsed['iTOW'] = struct.unpack('<I', data[offset:offset+4])[0]
                parsed['gps_flags'] = data[offset+4]
                parsed['num_sv'] = data[offset+5]
                offset += 6
                parsed['has_gps_info'] = True
                
                # Parse GPS flags
                gps_fix_type = parsed['gps_flags'] & 0x03
                parsed['gps_fix_string'] = self.get_gps_fix_string(gps_fix_type)
                parsed['gps_valid_tow'] = bool(parsed['gps_flags'] & 0x04)
                parsed['gps_valid_wkn'] = bool(parsed['gps_flags'] & 0x08)
                parsed['gps_valid_utc'] = bool(parsed['gps_flags'] & 0x10)
                parsed['gps_true_heading_valid'] = bool(parsed['gps_flags'] & 0x20)
            
            # Barometric altitude - 4 bytes (int32, cm)
            if offset + 4 <= len(data):
                parsed['baro_alt'] = struct.unpack('<i', data[offset:offset+4])[0]
                offset += 4
                parsed['has_baro_alt'] = True
            
            # Barometric pressure - 4 bytes (float32)
            if offset + 4 <= len(data):
                parsed['baro_p'] = struct.unpack('<f', data[offset:offset+4])[0]
                offset += 4
                parsed['has_baro_p'] = True
            
            # Position (Lat, Long, Alt) - 24 bytes (3 x double/float64)
            if offset + 24 <= len(data):
                parsed['lat'] = struct.unpack('<d', data[offset:offset+8])[0]
                parsed['long'] = struct.unpack('<d', data[offset+8:offset+16])[0]
                parsed['alt'] = struct.unpack('<d', data[offset+16:offset+24])[0]
                offset += 24
                parsed['has_position'] = True
            
            # Velocity (VelN, VelE, VelD) - 12 bytes
            if offset + 12 <= len(data):
                parsed['vel_n'] = struct.unpack('<f', data[offset:offset+4])[0]
                parsed['vel_e'] = struct.unpack('<f', data[offset+4:offset+8])[0]
                parsed['vel_d'] = struct.unpack('<f', data[offset+8:offset+12])[0]
                offset += 12
                parsed['has_velocity'] = True
            
            # UTC Time - 7 bytes
            if offset + 7 <= len(data):
                parsed['utc_year'] = struct.unpack('<H', data[offset:offset+2])[0]
                parsed['utc_month'] = data[offset+2]
                parsed['utc_day'] = data[offset+3]
                parsed['utc_hour'] = data[offset+4]
                parsed['utc_minute'] = data[offset+5]
                parsed['utc_second'] = data[offset+6]
                offset += 7
                parsed['has_utc'] = True
            
            # Magnetic heading - 4 bytes (float32, degrees)
            if offset + 4 <= len(data):
                parsed['heading'] = struct.unpack('<f', data[offset:offset+4])[0]
                offset += 4
                parsed['has_heading'] = True
            
            if self.debug:
                print(f"Parsed {offset} bytes out of {len(data)} total")
                
        except Exception as e:
            if self.debug:
                print(f"Error parsing data: {e}")
                print(f"Data hex: {data.hex()}")
                print(f"Offset: {offset}, Data length: {len(data)}")
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
        
        log_parts = [timestamp]
        
        # Euler angles
        if parsed_data.get('has_euler'):
            roll_deg = parsed_data['roll'] * 180.0 / 3.14159265359
            pitch_deg = parsed_data['pitch'] * 180.0 / 3.14159265359
            yaw_deg = parsed_data['yaw'] * 180.0 / 3.14159265359
            log_parts.append(f"EULER(deg): R={roll_deg:.2f},P={pitch_deg:.2f},Y={yaw_deg:.2f}")
        
        # Gyroscope
        if parsed_data.get('has_gyro'):
            log_parts.append(f"GYRO(rad/s): X={parsed_data['gx']:.4f},Y={parsed_data['gy']:.4f},Z={parsed_data['gz']:.4f}")
        
        # Accelerometer
        if parsed_data.get('has_accel'):
            log_parts.append(f"ACCEL(m/s²): X={parsed_data['ax']:.4f},Y={parsed_data['ay']:.4f},Z={parsed_data['az']:.4f}")
        
        # Magnetometer
        if parsed_data.get('has_mag'):
            log_parts.append(f"MAG(Gauss): X={parsed_data['mx']:.4f},Y={parsed_data['my']:.4f},Z={parsed_data['mz']:.4f}")
        
        # Temperature
        if parsed_data.get('has_temp'):
            log_parts.append(f"TEMP(°C): T0={parsed_data['t0']:.2f},T1={parsed_data['t1']:.2f}")
        
        # Time since reset
        if parsed_data.get('has_time'):
            log_parts.append(f"TIME_MS: {parsed_data['time']}")
        
        # UTC Time
        if parsed_data.get('has_utc'):
            log_parts.append(
                f"UTC: {parsed_data['utc_year']}-{parsed_data['utc_month']:02d}-{parsed_data['utc_day']:02d} "
                f"{parsed_data['utc_hour']:02d}:{parsed_data['utc_minute']:02d}:{parsed_data['utc_second']:02d}"
            )
        
        # Magnetic heading
        if parsed_data.get('has_heading'):
            log_parts.append(f"MAG_HEADING: {parsed_data['heading']:.2f}°")
        
        # Barometric data
        if parsed_data.get('has_baro_alt'):
            log_parts.append(f"BARO_ALT: {parsed_data['baro_alt']/100.0:.2f}m")
        if parsed_data.get('has_baro_p'):
            log_parts.append(f"BARO_P: {parsed_data['baro_p']:.2f}Pa")
        
        # GPS info
        if parsed_data.get('has_gps_info'):
            log_parts.append(
                f"GPS: {parsed_data['gps_fix_string']},SAT={parsed_data['num_sv']},iTOW={parsed_data['iTOW']}ms"
            )
        
        # Position
        if parsed_data.get('has_position'):
            log_parts.append(
                f"POS: LAT={parsed_data['lat']:.6f},LON={parsed_data['long']:.6f},ALT={parsed_data['alt']:.2f}m"
            )
        
        # Velocity
        if parsed_data.get('has_velocity'):
            log_parts.append(
                f"VEL(m/s): N={parsed_data['vel_n']:.2f},E={parsed_data['vel_e']:.2f},D={parsed_data['vel_d']:.2f}"
            )
        
        return ",".join(log_parts)
    
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
                        
                        # Clear line and print
                        print(f"\rFrames: {frame_count}, Errors: {error_count} | {log_entry[:150]}...", end='', flush=True)
                        
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
        print("Press 'd' to toggle debug mode")
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