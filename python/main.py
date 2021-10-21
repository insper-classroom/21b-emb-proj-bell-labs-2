import sys
import serial

from converter.converter import Converter

START_CHAR = 'init'
END_CHAR = '-=-=-=-=-=-=-=-=-=-=-'

if __name__ == "__main__":
    if (len(sys.argv) == 1):
        print("ERROR: No COM argument.")
        exit(0)
    elif (len(sys.argv) == 2):
        print("ERROR: No Baudrate argument.")
        exit(0)
    
    com = sys.argv[1]
    baudrate = sys.argv[2]

    serial = serial.Serial(port = com, baudrate = baudrate)
    converter = Converter()

    audio = list()

    while True:
        data = serial.readline().decode('ASCII').replace('\n', '')
        if (data == START_CHAR):
            data = serial.readline().decode('ASCII').replace('\n', '')
            while (data != END_CHAR):
                audio.append(data)
                data = serial.readline().decode('ASCII').replace('\n', '')
            
            converter.convert_audio(audio)