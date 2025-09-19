#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import argparse

def parse_log_file(filename):
    """Parse the IMU log file into a pandas DataFrame"""
    data = []
    
    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) >= 7:
                # Extract timestamp
                timestamp = datetime.strptime(parts[0], "%Y-%m-%d %H:%M:%S.%f")
                
                # Extract Euler angles
                euler = parts[1].split(':')[1]
                roll = float(euler.split('=')[1].split(',')[0])
                pitch = float(parts[1].split('P=')[1].split(',')[0])
                yaw = float(parts[1].split('Y=')[1])
                
                # Extract other values
                heading = float(parts[3].split(':')[1].replace('°', ''))
                altitude = float(parts[4].split(':')[1].replace('m', ''))
                
                # Extract GPS info
                gps_parts = parts[5].split(':')[1].split(',')
                gps_fix = gps_parts[0]
                satellites = int(gps_parts[1].split('=')[1])
                
                # Extract position
                lat = float(parts[6].split('LAT=')[1].split(',')[0])
                lon = float(parts[6].split('LON=')[1].split(',')[0])
                
                data.append({
                    'timestamp': timestamp,
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw,
                    'heading': heading,
                    'altitude': altitude,
                    'gps_fix': gps_fix,
                    'satellites': satellites,
                    'latitude': lat,
                    'longitude': lon
                })
    
    return pd.DataFrame(data)

def plot_angles(df):
    """Plot Euler angles over time"""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    
    ax1.plot(df['timestamp'], df['roll'])
    ax1.set_ylabel('Roll (deg)')
    ax1.grid(True)
    
    ax2.plot(df['timestamp'], df['pitch'])
    ax2.set_ylabel('Pitch (deg)')
    ax2.grid(True)
    
    ax3.plot(df['timestamp'], df['yaw'])
    ax3.set_ylabel('Yaw (deg)')
    ax3.set_xlabel('Time')
    ax3.grid(True)
    
    plt.suptitle('IMU Euler Angles')
    plt.tight_layout()
    plt.show()

def plot_position(df):
    """Plot GPS position"""
    plt.figure(figsize=(10, 8))
    plt.scatter(df['longitude'], df['latitude'], c=df['satellites'], cmap='viridis')
    plt.colorbar(label='Number of Satellites')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('GPS Track')
    plt.grid(True)
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='View IMU log data')
    parser.add_argument('logfile', help='Log file to analyze')
    parser.add_argument('--plot', choices=['angles', 'position', 'both'], 
                       default='both', help='What to plot')
    args = parser.parse_args()
    
    print(f"Loading {args.logfile}...")
    df = parse_log_file(args.logfile)
    print(f"Loaded {len(df)} entries")
    
    # Display summary
    print("\nData Summary:")
    print(f"Time range: {df['timestamp'].min()} to {df['timestamp'].max()}")
    print(f"Average satellites: {df['satellites'].mean():.1f}")
    print(f"Altitude range: {df['altitude'].min():.1f}m to {df['altitude'].max():.1f}m")
    
    if args.plot in ['angles', 'both']:
        plot_angles(df)
    
    if args.plot in ['position', 'both']:
        plot_position(df)

if __name__ == "__main__":
    main()