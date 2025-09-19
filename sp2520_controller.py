import serial

class SP2520Controller:
    def __init__(self, port='/dev/ttyUSB1', baudrate=2400, address=1):
        self.address = address
        self.pan_speed = 0x20
        self.tilt_speed = 0x20
        
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
    
    def send_command(self, cmd1, cmd2, data1, data2):
        checksum = (self.address + cmd1 + cmd2 + data1 + data2) % 256
        command = bytes([0xFF, self.address, cmd1, cmd2, data1, data2, checksum])
        self.serial.write(command)
    
    def stop(self):
        self.send_command(0x00, 0x00, 0x00, 0x00)
    
    def move_left(self):
        self.send_command(0x00, 0x04, self.pan_speed, 0x00)
    
    def move_right(self):
        self.send_command(0x00, 0x02, self.pan_speed, 0x00)
    
    def move_up(self):
        self.send_command(0x00, 0x08, 0x00, self.tilt_speed)
    
    def move_down(self):
        self.send_command(0x00, 0x10, 0x00, self.tilt_speed)
    
    def move_up_left(self):
        self.send_command(0x00, 0x0C, self.pan_speed, self.tilt_speed)
    
    def move_up_right(self):
        self.send_command(0x00, 0x0A, self.pan_speed, self.tilt_speed)
    
    def move_down_left(self):
        self.send_command(0x00, 0x14, self.pan_speed, self.tilt_speed)
    
    def move_down_right(self):
        self.send_command(0x00, 0x12, self.pan_speed, self.tilt_speed)
    
    def set_speed(self, pan_speed=None, tilt_speed=None):
        if pan_speed is not None:
            self.pan_speed = max(0, min(0x40, pan_speed))
        if tilt_speed is not None:
            self.tilt_speed = max(0, min(0x40, tilt_speed))
    
    def call_preset(self, preset_num):
        self.send_command(0x00, 0x07, 0x00, preset_num)
    
    def set_preset(self, preset_num):
        self.send_command(0x00, 0x03, 0x00, preset_num)
    
    def close(self):
        if self.serial and self.serial.is_open:
            self.stop()
            self.serial.close()

    