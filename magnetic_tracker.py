#!/usr/bin/env python3
"""
Magnetic Heading Tracker
Reads magnetic heading from IMU560 and controls SP2520 PTZ to follow heading
"""

import time
import threading
import configparser
from collections import deque
import datetime
import sys
import os

# Import the IMU reader and SP2520 controller
from imu_reader import IMU560Reader
from sp2520_controller import SP2520Controller

class MagneticTracker:
    def __init__(self, config_file='config.ini'):
        # Read configuration
        self.config = configparser.ConfigParser()
        self.config.read(config_file)
        
        # Tracking configuration
        self.target_tilt = self.config.getfloat('SP2520', 'target_tilt')
        self.heading_offset = self.config.getfloat('SP2520', 'heading_offset')
        self.dead_zone = self.config.getfloat('SP2520', 'dead_zone')
        self.update_rate = self.config.getfloat('SP2520', 'update_rate')
        
        # Initialize components
        self.imu_reader = IMU560Reader(config_file)
        self.ptz_controller = SP2520Controller(config_file)
        
        # State variables
        self.running = False
        self.current_heading = 0.0
        self.current_tilt = 0.0
        self.target_heading = 0.0
        self.last_imu_data = None
        
        # Statistics
        self.stats = {
            'imu_frames': 0,
            'tracking_updates': 0,
            'errors': 0
        }
        
        # Moving average for heading (reduces jitter)
        self.heading_buffer = deque(maxlen=5)
        
    def normalize_angle(self, angle):
        """Normalize angle to -180 to +180 range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def calculate_angle_difference(self, target, current):
        """Calculate shortest angular distance between two angles"""
        diff = self.normalize_angle(target - current)
        return diff
    
    def get_smoothed_heading(self):
        """Get average heading from buffer"""
        if not self.heading_buffer:
            return self.current_heading
        return sum(self.heading_buffer) / len(self.heading_buffer)
    
    def imu_data_callback(self, parsed_data):
        """Callback for IMU data - override the IMU reader's display"""
        if parsed_data and 'heading' in parsed_data:
            # Get magnetic heading (-180 to +180)
            raw_heading = parsed_data['heading']
            
            # Apply offset
            adjusted_heading = self.normalize_angle(raw_heading + self.heading_offset)
            
            # Add to buffer for smoothing
            self.heading_buffer.append(adjusted_heading)
            self.current_heading = self.get_smoothed_heading()
            
            self.last_imu_data = parsed_data
            self.stats['imu_frames'] += 1
    
    def tracking_thread(self):
        """Main tracking control thread"""
        last_update = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # Update at specified rate
                if current_time - last_update >= 1.0 / self.update_rate:
                    last_update = current_time
                    
                    if self.last_imu_data is not None:
                        # Calculate heading error
                        heading_error = self.calculate_angle_difference(
                            self.target_heading, self.current_heading
                        )
                        
                        # Control logic
                        if abs(heading_error) > self.dead_zone:
                            # Determine direction and move
                            if heading_error > 0:
                                self.ptz_controller.move_right()
                            else:
                                self.ptz_controller.move_left()
                            
                            # Brief movement based on error magnitude
                            move_time = min(0.1 * (abs(heading_error) / 10), 0.5)
                            time.sleep(move_time)
                            self.ptz_controller.stop()
                            
                            self.stats['tracking_updates'] += 1
                
                time.sleep(0.01)  # Small sleep to prevent CPU hogging
                
            except Exception as e:
                self.stats['errors'] += 1
                print(f"\nError in tracking thread: {e}")
                time.sleep(0.1)
    
    def display_thread(self):
        """Display status information"""
        while self.running:
            try:
                timestamp = datetime.datetime.now().strftime("%H:%M:%S")
                
                # Build status line
                status = (
                    f"\r[{timestamp}] "
                    f"Heading: {self.current_heading:+07.2f}° | "
                    f"Target: {self.target_heading:+07.2f}° | "
                    f"Error: {self.calculate_angle_difference(self.target_heading, self.current_heading):+06.2f}° | "
                    f"IMU Frames: {self.stats['imu_frames']:5d} | "
                    f"Updates: {self.stats['tracking_updates']:5d} | "
                    f"Errors: {self.stats['errors']:3d}"
                )
                
                print(status, end='', flush=True)
                
                time.sleep(0.1)
                
            except Exception as e:
                print(f"\nDisplay error: {e}")
                time.sleep(1)
    
    def start(self):
        """Start the magnetic tracker"""
        print("Starting Magnetic Heading Tracker...")
        print(f"Target Tilt: {self.target_tilt}°")
        print(f"Heading Offset: {self.heading_offset}°")
        print(f"Dead Zone: {self.dead_zone}°")
        print(f"Update Rate: {self.update_rate} Hz")
        print("-" * 80)
        
        # Override IMU reader's format_log_entry to capture data
        original_format = self.imu_reader.format_log_entry
        
        def new_format(parsed_data):
            self.imu_data_callback(parsed_data)
            return original_format(parsed_data)
        
        self.imu_reader.format_log_entry = new_format
        
        # Start IMU reader
        if not self.imu_reader.start():
            print("Failed to start IMU reader")
            return False
        
        # Home the PTZ (optional - set to a known position)
        print("Homing PTZ...")
        self.ptz_controller.stop()
        time.sleep(1)
        
        # Start tracking
        self.running = True
        self.tracking_thread_obj = threading.Thread(target=self.tracking_thread)
        self.display_thread_obj = threading.Thread(target=self.display_thread)
        
        self.tracking_thread_obj.start()
        self.display_thread_obj.start()
        
        print("\nTracking started. Press 't' to set new target, 'h' to home, 'q' to quit")
        return True
    
    def stop(self):
        """Stop the tracker"""
        print("\n\nStopping tracker...")
        self.running = False
        
        # Stop threads
        if hasattr(self, 'tracking_thread_obj'):
            self.tracking_thread_obj.join()
        if hasattr(self, 'display_thread_obj'):
            self.display_thread_obj.join()
        
        # Stop components
        self.ptz_controller.stop()
        self.ptz_controller.close()
        self.imu_reader.stop()
        
        print("Tracker stopped.")
    
    def set_target_heading(self, heading):
        """Set new target heading"""
        self.target_heading = self.normalize_angle(heading)
        print(f"\nNew target heading: {self.target_heading}°")
    
    def home_position(self):
        """Return to home position (0 degrees)"""
        self.set_target_heading(0)
        print("\nReturning to home position...")

def main():
    tracker = MagneticTracker('config.ini')
    
    try:
        if tracker.start():
            while True:
                # Check for user input
                import select
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1).lower()
                    
                    if key == 'q':
                        break
                    elif key == 't':
                        # Set new target
                        print("\nEnter new target heading (-180 to +180): ", end='', flush=True)
                        try:
                            new_heading = float(input())
                            if -180 <= new_heading <= 180:
                                tracker.set_target_heading(new_heading)
                            else:
                                print("Heading must be between -180 and +180 degrees")
                        except ValueError:
                            print("Invalid input")
                    elif key == 'h':
                        # Home position
                        tracker.home_position()
                    elif key == 's':
                        # Show statistics
                        print(f"\n\nStatistics:")
                        print(f"  IMU Frames: {tracker.stats['imu_frames']}")
                        print(f"  Tracking Updates: {tracker.stats['tracking_updates']}")
                        print(f"  Errors: {tracker.stats['errors']}")
                        print(f"  Current Heading: {tracker.current_heading:.2f}°")
                        print(f"  Target Heading: {tracker.target_heading:.2f}°")
                        print()
                    elif key == 'z':
                        # Zero heading offset
                        print(f"\nCurrent heading: {tracker.current_heading:.2f}°")
                        print("Setting this as zero reference...")
                        tracker.heading_offset = -tracker.current_heading
                        print(f"New heading offset: {tracker.heading_offset:.2f}°")
                
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop()

if __name__ == "__main__":
    main()