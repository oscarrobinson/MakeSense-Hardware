import serial.tools.list_ports

print(list(serial.tools.list_ports.comports()))

# currently plugged into /dev/ttyUSB0  <- right hand side closest to CD
