import serial
import struct
import datetime
import configparser
import os
import time
import threading
from collections import deque

class IMU560Reader:
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
        
        # Data structure offsets (in bytes)
        self.data_offsets = {
            'roll': 0,
            'pitch': 4,
            'yaw': 8,
            'gx': 12,
            'gy': 16,
            'gz': 20,
            'ax': 24,
            'ay': 28,
            'az': 32,
            'mx': 36,
            'my': 40,
            'mz': 44,
            't0': 48,
            't1': 52,
            'time': 56,
            'iTOW': 60,
            'gps_flags': 64,
            'num_sv': 65,
            'baro_alt': 66,
            'baro_p': 70,
            'lat': 74,
            'long': 82,
            'alt': 90,
            'vel_n': 98,
            'vel_e': 102,
            'vel_d': 106,
            'utc': 110,
            'heading': 117
        }
        
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
        """Calculate CRC16-XMODEM (common for IMU devices)"""
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
        """Calculate CRC16-MODBUS (alternative algorithm)"""
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
            print(f"Data length: {data_length}")
        
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
        
        # Try different CRC calculations
        received_crc = struct.unpack('>H', crc_bytes)[0]
        
        # Include different parts of the frame in CRC calculation
        # Method 1: Only data
        crc1 = self.calculate_crc16_modbus(data)
        
        # Method 2: CMD + Length + Data
        crc2 = self.calculate_crc16_modbus(cmd + len_bytes + data)
        
        # Method 3: Length + Data
        crc3 = self.calculate_crc16_modbus(len_bytes + data)
        
        # Method 4: Using XMODEM algorithm
        crc4 = self.calculate_crc16_xmodem(cmd + len_bytes + data)
        
        if self.debug:
            print(f"Received CRC: 0x{received_crc:04X}")
            print(f"CRC Method 1 (data only): 0x{crc1:04X}")
            print(f"CRC Method 2 (cmd+len+data): 0x{crc2:04X}")
            print(f"CRC Method 3 (len+data): 0x{crc3:04X}")
            print(f"CRC Method 4 (xmodem): 0x{crc4:04X}")
        
        # For now, skip CRC verification if none match
        # You can enable strict CRC checking once we identify the correct method
        if received_crc not in [crc1, crc2, crc3, crc4]:
            if self.debug:
                print("CRC mismatch - proceeding anyway")
            # Uncomment below to enforce CRC checking
            # return None
        
        return data
    
    def parse_data(self, data):
        """Parse the data buffer according to IMU560 format"""
        parsed = {}
        
        # Expected data length is 121 bytes according to your documentation
        if len(data) != 121:
            if self.debug:
                print(f"Warning: Data length is {len(data)}, expected 121")
        
        try:
            # Attitude angles (Euler) - float32, radians
            parsed['roll'] = struct.unpack('<f', data[self.data_offsets['roll']:self.data_offsets['roll']+4])[0]
            parsed['pitch'] = struct.unpack('<f', data[self.data_offsets['pitch']:self.data_offsets['pitch']+4])[0]
            parsed['yaw'] = struct.unpack('<f', data[self.data_offsets['yaw']:self.data_offsets['yaw']+4])[0]
            
            # Gyroscope data
            parsed['gx'] = struct.unpack('<f', data[self.data_offsets['gx']:self.data_offsets['gx']+4])[0]
            parsed['gy'] = struct.unpack('<f', data[self.data_offsets['gy']:self.data_offsets['gy']+4])[0]
            parsed['gz'] = struct.unpack('<f', data[self.data_offsets['gz']:self.data_offsets['gz']+4])[0]
            
            # Accelerometer data
            parsed['ax'] = struct.unpack('<f', data[self.data_offsets['ax']:self.data_offsets['ax']+4])[0]
            parsed['ay'] = struct.unpack('<f', data[self.data_offsets['ay']:self.data_offsets['ay']+4])[0]
            parsed['az'] = struct.unpack('<f', data[self.data_offsets['az']:self.data_offsets['az']+4])[0]
            
            # GPS Info
            parsed['iTOW'] = struct.unpack('<I', data[self.data_offsets['iTOW']:self.data_offsets['iTOW']+4])[0]
            parsed['gps_flags'] = data[self.data_offsets['gps_flags']]
            parsed['num_sv'] = data[self.data_offsets['num_sv']]
            
            # Barometric altitude - int32, cm
            parsed['baro_alt'] = struct.unpack('<i', data[self.data_offsets['baro_alt']:self.data_offsets['baro_alt']+4])[0]
            
            # Position - double (float64), degrees and meters
            parsed['lat'] = struct.unpack('<d', data[self.data_offsets['lat']:self.data_offsets['lat']+8])[0]
            parsed['long'] = struct.unpack('<d', data[self.data_offsets['long']:self.data_offsets['long']+8])[0]
            parsed['alt'] = struct.unpack('<d', data[self.data_offsets['alt']:self.data_offsets['alt']+8])[0]
            
            # UTC Time
            utc_data = data[self.data_offsets['utc']:self.data_offsets['utc']+7]
            parsed['utc_year'] = struct.unpack('<H', utc_data[0:2])[0]
            parsed['utc_month'] = utc_data[2]
            parsed['utc_day'] = utc_data[3]
            parsed['utc_hour'] = utc_data[4]
            parsed['utc_minute'] = utc_data[5]
            parsed['utc_second'] = utc_data[6]
            
            # Magnetic heading - float32, degrees
            parsed['heading'] = struct.unpack('<f', data[self.data_offsets['heading']:self.data_offsets['heading']+4])[0]
            
            # GPS status parsing
            gps_fix_type = parsed['gps_flags'] & 0x03
            parsed['gps_fix_string'] = self.get_gps_fix_string(gps_fix_type)
            parsed['gps_valid_tow'] = bool(parsed['gps_flags'] & 0x04)
            parsed['gps_valid_wkn'] = bool(parsed['gps_flags'] & 0x08)
            parsed['gps_valid_utc'] = bool(parsed['gps_flags'] & 0x10)
            parsed['gps_true_heading_valid'] = bool(parsed['gps_flags'] & 0x20)
            
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
        
        # Convert radians to degrees for display
        roll_deg = parsed_data['roll'] * 180.0 / 3.14159265359
        pitch_deg = parsed_data['pitch'] * 180.0 / 3.14159265359
        yaw_deg = parsed_data['yaw'] * 180.0 / 3.14159265359
        
        log_entry = (
            f"{timestamp},"
            f"EULER(deg): R={roll_deg:.2f},P={pitch_deg:.2f},Y={yaw_deg:.2f},"
            f"UTC: {parsed_data['utc_year']}-{parsed_data['utc_month']:02d}-{parsed_data['utc_day']:02d} "
            f"{parsed_data['utc_hour']:02d}:{parsed_data['utc_minute']:02d}:{parsed_data['utc_second']:02d},"
            f"MAG_HEADING: {parsed_data['heading']:.2f}°,"
            f"BARO_ALT: {parsed_data['baro_alt']/100.0:.2f}m,"
            f"GPS: {parsed_data['gps_fix_string']},SAT={parsed_data['num_sv']},iTOW={parsed_data['iTOW']}ms,"
            f"POS: LAT={parsed_data['lat']:.6f},LON={parsed_data['long']:.6f},ALT={parsed_data['alt']:.2f}m"
        )
        
        return log_entry
    
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
                        print(f"\rFrames: {frame_count}, Errors: {error_count} | {log_entry[:100]}...", end='', flush=True)
                        
                        # Write to file periodically (every 100 entries)
                        if len(self.data_buffer) % 100 == 0:
                            self.write_log()
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
    reader = IMU560Reader('config.ini')
    
    # Check if we need to run in debug mode
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == '--debug':
        reader.debug = True
        print("Starting in debug mode")
    
    try:
        if reader.start():
            while True:
                # Non-blocking keyboard input check
                import select
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 'd':
                        reader.toggle_debug()
                time.sleep(0.1)
    except KeyboardInterrupt:
        reader.stop()

if __name__ == "__main__":
    main()