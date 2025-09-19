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
    
    def calculate_crc(self, data):
        """Calculate CRC16 for data verification"""
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
        while True:
            byte = self.serial_port.read(1)
            if not byte:
                return False
            
            if byte[0] == self.SYNC_BYTE:
                next_byte = self.serial_port.read(1)
                if not next_byte:
                    return False
                
                if next_byte[0] == self.START_BYTE:
                    return True
        
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
        
        # Read data
        data = self.serial_port.read(data_length)
        if len(data) != data_length:
            return None
        
        # Read CRC (2 bytes)
        crc_bytes = self.serial_port.read(2)
        if len(crc_bytes) != 2:
            return None
        
        # Read end byte
        end_byte = self.serial_port.read(1)
        if not end_byte or end_byte[0] != self.END_BYTE:
            return None
        
        # Verify CRC
        received_crc = struct.unpack('>H', crc_bytes)[0]
        calculated_crc = self.calculate_crc(cmd + len_bytes + data)
        
        if received_crc != calculated_crc:
            print("CRC mismatch!")
            return None
        
        return data
    
    def parse_data(self, data):
        """Parse the data buffer according to IMU560 format"""
        parsed = {}
        
        try:
            # Attitude angles (Euler) - float32, radians
            parsed['roll'] = struct.unpack('<f', data[self.data_offsets['roll']:self.data_offsets['roll']+4])[0]
            parsed['pitch'] = struct.unpack('<f', data[self.data_offsets['pitch']:self.data_offsets['pitch']+4])[0]
            parsed['yaw'] = struct.unpack('<f', data[self.data_offsets['yaw']:self.data_offsets['yaw']+4])[0]
            
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
            print(f"Error parsing data: {e}")
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
        while self.running:
            try:
                frame_data = self.read_frame()
                if frame_data:
                    parsed_data = self.parse_data(frame_data)
                    if parsed_data:
                        log_entry = self.format_log_entry(parsed_data)
                        self.data_buffer.append(log_entry)
                        print(f"\r{log_entry}", end='', flush=True)
                        
                        # Write to file periodically (every 100 entries)
                        if len(self.data_buffer) % 100 == 0:
                            self.write_log()
                
            except Exception as e:
                print(f"\nError in reading thread: {e}")
                time.sleep(0.1)
    
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
    reader = IMU560Reader('config.ini')
    
    try:
        if reader.start():
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        reader.stop()

if __name__ == "__main__":
    main()