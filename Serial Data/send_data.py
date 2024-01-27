import serial
import time

#opening serial comm
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) 
			# port name, baud rate, timeout(as SECONDS)
time.sleep(2) # wait for the connection to establish

try:
    while True:
        command = input("write the text: ")
        command = command + '\n'     #'encode function only encodes our text, it doesn't add a \n, we should manually add \n to packages'
        ser.write(command.encode())  # Encode and send command
except KeyboardInterrupt:
    ser.close()  # Close serial connection when done

