import serial 
import numpy as np 

ser = serial.Serial("/dev/ttyArduino", 1000000)
print(ser.name)
def read(size=1, timeout=None):
    ser.timeout = timeout
    readed = ser.read(size)
    return readed
command = b"\xfa\xfe\x03\x01\x04\xfa\xfd"
ser.write(command)

incoming = read(size=20, timeout=1)
print(incoming)

command = b"\xfa\xfe\x01\x01\x01\x03\x06\xfa\xfd"
ser.write(command)

print(read(size=20, timeout=1))

command = [0xfa, 0xfe, 2, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0xfa, 0xfd]

command[3] = 0
command[4] = 250
command[5] = 250 
command[6] = 10
command[7] = 250
command[8] = 255
command[9] = 255
command[12] = np.uint8(sum(command[2:12])) 

print(command)
print(bytes(command))

ser.write(bytes(command))