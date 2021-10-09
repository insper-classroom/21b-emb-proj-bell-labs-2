import serial

serialPort = serial.Serial(port="COM3", baudrate=115200, bytesize=8)

while(True):

	lista.append(serialPort.readline())