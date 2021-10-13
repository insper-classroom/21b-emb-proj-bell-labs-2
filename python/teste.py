import serial
import time

serialPort = serial.Serial(port="COM3", baudrate=115200, bytesize=8, timeout=None, stopbits=1)
start = time.time()
lista = list()

while(time.time() - start < 3):

	lista.append(serialPort.readline())

with open("xibiu.txt", "a+") as cu:
	for i in lista:
		i = i.decode("Ascii").replace("\n", "")
		cu.write(f"{i}*")