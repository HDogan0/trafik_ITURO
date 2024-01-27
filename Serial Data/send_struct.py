import serial
import struct
import time

#SENDING DATA AS A STRUCT

class struct:
    def __init__(self, sleep_time):
        self.sleep_time = sleep_time

    def to_bytes(self):
        return struct.pack('i', self.sleep_time)

#proper port name and baudrate for serial comm, (timeout as SECONDS)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  

data = struct(100)

try:
    while True:
        data.sleep_time = int(input("sleep time: "))
        command = data.to_bytes()
	ser.write(command)
except KeyboardInterrupt:
    ser.close()  
