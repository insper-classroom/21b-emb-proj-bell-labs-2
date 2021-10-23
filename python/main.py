import os
import sys
import signal
import serial
import time

import numpy as np
from scipy.io.wavfile import write
import soundfile as sf

from google.cloud import speech

os.environ['GOOGLE_APPLICATION_CREDENTIALS']= 'google_credentials.json'

TS = 11000
START_CHAR = 'A'
END_CHAR = 'X'

client = speech.SpeechClient()

def convert(data):

    audio = np.array(data, dtype=np.int16)
    audio = 3.3*audio/4095

    for file in os.listdir():
        if(file == "output.flac"):
            os.remove(file)

    sf.write('output.flac', audio, TS)

def transcript_google():

    print("Starting transcript...")

    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.FLAC,
        sample_rate_hertz=TS,
        language_code="pt-BR",
    )

    with open("output.flac", 'rb') as f:
        content = f.read()

    audio = { "content": content }

    # Detects speech in the audio file
    response = client.recognize(config=config, audio=audio)

    for result in response.results:
        print("Transcript: {}".format(result.alternatives[0].transcript))

def handler():
    res = input("Do you want to exit? y/n ")
    if res == 'y': 
        exit(1)

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

    audio = list()

    while True:
        print(f'Waiting for data on {com} w/ baudrate {baudrate}...')
        data = serial.readline().decode('ASCII').replace('\n', '')
        if (data == START_CHAR):
            data = serial.readline().decode('ASCII').replace('\n', '')
            while (data != END_CHAR):
                audio.append(data)
                data = serial.readline().decode('ASCII').replace('\n', '')
            
            print(f'Received {len(audio)} entries')
            convert(audio)
            transcript_google()
            handler()

        time.sleep(5)