To make the USB port accessible without sudo on Raspberry Pi:

# Add your user to the dialout group
sudo usermod -a -G dialout $USER

# Create a udev rule for the IMU
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"' | sudo tee /etc/udev/rules.d/99-imu.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Logout and login again for group change to take effect