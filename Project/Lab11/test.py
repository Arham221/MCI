import serial
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
for _ in range(10):
    print(s.readline())