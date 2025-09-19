#!/usr/bin/env python3
import serial
import struct
import datetime
import configparser
import time
import threading
from collections import deque

class IMU560ReaderNoCRC:
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
        self.EXPECTED_DATA_LENGTH = 121  # Based on documentation
        
        # Serial connection
        self.serial_port = None
        self.running = False
        
        # Statistics
        self.frames_received = 0
        self.frames_error = 0
        
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
    
    def find_sync(self):
        """Find frame synchronization"""
        while self.running:
            b1 = self.serial_port.read(1)
            if not b1:
                continue
                
            if b1[0] == self.SYNC_BYTE:
                b2 = self.serial_port.read(1)
                if b2 and b2[0] == self.START_BYTE:
                    return True
        return False
    
    def read_frame_simple(self):
        """Read frame without CRC verification"""
        # Find sync
        if not self.find_sync():
            return None
        
        # Read CMD
        cmd = self.serial_port.read(1)
        if not cmd or cmd[0] != self.CMD_CONTINUOUS:
            self.frames_error += 1
            return None
        
        # Read length
        len_bytes = self.serial_port.read(2)
        if len(len_bytes) != 2:
            self.frames_error += 1
            return None
        
        data_length = struct.unpack('>H', len_bytes)[0]
        
        # Sanity check on length
        if data_length != self.EXPECTED_DATA_LENGTH:
            print(f"\nWarning: Unexpected data length {data_length}, expected {self.EXPECTED_DATA_LENGTH}")
        
        # Read data
        data = self.serial_port.read(data_length)
        if len(data) != data_length:
            self.frames_error += 1
            return None
        
        # Skip CRC (2 bytes)
        self.serial_port.read(2)
        
        # Read end byte
        end = self.serial_port.read(1)
        if not end or end[0] != self.END_BYTE:
            self.frames_error += 1
            return None
        
        self.frames_received += 1
        return data
    
    def parse_minimal(self, data):
        """Parse only the requested fields"""
        try:
            parsed = {}
            
            # 1. Euler angles (we need all three but focus on yaw)
            parsed['roll'] = struct.unpack('<f', data[0:4])[0] * 180.0 / 3.14159265359
            parsed['pitch'] = struct.unpack('<f', data[4:8])[0] * 180.0 / 3.14159265359
            parsed['yaw'] = struct.unpack('<f', data[8:12])[0] * 180.0 / 3.14159265359
            
            # 2. UTC Time (offset 110)
            utc_year = struct.unpack('<H', data[110:112])[0]
            utc_month = data[112]
            utc_day = data[113]
            utc_hour = data[114]
            utc_minute = data[115]
            utc_second = data[116]
            parsed['utc_time'] = f"{utc_year:04d}-{utc_month:02d}-{utc_day:02d} {utc_hour:02d}:{utc_minute:02d}:{utc_second:02d}"
            
            # 3. Magnetic heading (offset 117)
            parsed['heading'] = struct.unpack('<f', data[117:121])[0]
            
            # 4. Barometric altitude (offset 66)
            baro_alt_cm = struct.unpack('<i', data[66:70])[0]
            parsed['baro_alt'] = baro_alt_cm / 100.0  # Convert to meters
            
            # 5. GPS info (offset 60)
            parsed['iTOW'] = struct.unpack('<I', data[60:64])[0]
            parsed['gps_flags'] = data[64]
            parsed['num_sv'] = data[65]
            
            # Parse GPS fix type
            gps_fix = parsed['gps_flags'] & 0x03
            fix_types = {0: "NO_FIX", 1: "TIME_ONLY", 2: "2D_FIX", 3: "3D_FIX"}
            parsed['gps_fix'] = fix_types.get(gps_fix, "UNKNOWN")
            
            # 6. Position (offset 74)
            parsed['lat'] = struct.unpack('<d', data[74:82])[0]
            parsed['lon'] = struct.unpack('<d', data[82:90])[0]
            parsed['alt'] = struct.unpack('<d', data[90:98])[0]
            
            return parsed
            
        except Exception as e:
            print(f"\nError parsing data: {e}")
            return None
    
    def format_log_entry(self, parsed):
        """Format data for logging"""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        log_entry = (
            f"{timestamp},"
            f"YAW={parsed['yaw']:.2f}°,"
            f"ROLL={parsed['roll']:.2f}°,"
            f"PITCH={parsed['pitch']:.2f}°,"
            f"UTC={parsed['utc_time']},"
            f"HDG={parsed['heading']:.2f}°,"
            f"BARO={parsed['baro_alt']:.2f}m,"
            f"GPS={parsed['gps_fix']},"
            f"SAT={parsed['num_sv']},"
            f"LAT={parsed['lat']:.6f},"
            f"LON={parsed['lon']:.6f},"
            f"ALT={parsed['alt']:.2f}m"
        )
        
        return log_entry
    
    def write_log(self):
        """Write buffer to file"""
        try:
            with open(self.log_file, 'w') as f:
                f.write("Timestamp,Yaw,Roll,Pitch,UTC_Time,Heading,Baro_Alt,GPS_Fix,Satellites,Latitude,Longitude,Altitude\n")
                for entry in self.data_buffer:
                    f.write(entry + '\n')
        except Exception as e:
            print(f"\nError writing log: {e}")
    
    def reading_loop(self):
        """Main reading loop"""
        last_save_time = time.time()
        
        while self.running:
            try:
                # Read frame
                data = self.read_frame_simple()
                if not data:
                    continue
                
                # Parse data
                parsed = self.parse_minimal(data)
                if not parsed:
                    continue
                
                # Format and store
                log_entry = self.format_log_entry(parsed)
                self.data_buffer.append(log_entry)
                
                # Display
                status = f"\rFrames: {self.frames_received} | Errors: {self.frames_error} | "
                status += f"Yaw: {parsed['yaw']:6.1f}° | "
                status += f"GPS: {parsed['gps_fix']} ({parsed['num_sv']} sats) | "
                status += f"Alt: {parsed['baro_alt']:6.1f}m"
                print(status, end='', flush=True)
                
                # Periodic save (every 10 seconds)
                if time.time() - last_save_time > 10:
                    self.write_log()
                    last_save_time = time.time()
                    
            except Exception as e:
                self.frames_error += 1
                print(f"\nError in reading loop: {e}")
                time.sleep(0.1)
    
    def run(self):
        """Main run method"""
        if not self.connect():
            return
        
        print("Starting IMU reader (No CRC verification)")
        print("Press Ctrl+C to stop\n")
        
        self.running = True
        
        try:
            self.reading_loop()
        except KeyboardInterrupt:
            print("\n\nStopping...")
        finally:
            self.running = False
            self.write_log()
            if self.serial_port:
                self.serial_port.close()
            print(f"Data saved to {self.log_file}")
            print(f"Total frames: {self.frames_received}, Errors: {self.frames_error}")

def main():
    reader = IMU560ReaderNoCRC('config.ini')
    reader.run()

if __name__ == "__main__":
    main()