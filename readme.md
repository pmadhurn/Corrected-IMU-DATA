To make the USB port accessible without sudo on Raspberry Pi:

# Add your user to the dialout group
sudo usermod -a -G dialout $USER

# Create a udev rule for the IMU
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"' | sudo tee /etc/udev/rules.d/99-imu.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Logout and login again for group change to take effect

To use these scripts:

First, test basic connectivity:
bash
python3 test_connection.py /dev/ttyUSB0
Run diagnostics to understand the frame format:
bash
python3 imu_diagnostic.py /dev/ttyUSB0
Use the no-CRC version to start logging data:
bash
python3 imu_reader_nocrc.py
The no-CRC version should work immediately and start displaying your IMU data. Once we understand the exact CRC algorithm from the diagnostic output, we can update the main reader with proper CRC verification.