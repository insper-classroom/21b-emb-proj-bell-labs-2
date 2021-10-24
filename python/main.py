# Imports

import os
import sys
import signal
import serial
import time

import numpy as np
from scipy.io.wavfile import write
from pydub import AudioSegment

from google.cloud import speech
import nltk
from nltk.stem.snowball import SnowballStemmer

# Autenticação

os.environ['GOOGLE_APPLICATION_CREDENTIALS']= 'google_credentials.json'

# Variáveis globais

TS = 11000
START_CHAR = 'A'
END_CHAR = 'X'

client = speech.SpeechClient()
stemmer = SnowballStemmer('portuguese')

# Função para converter para audio

def convert(data):

    audio = np.array(data, dtype=np.int16)

    write('output.wav', TS, audio)

# Função de transcrição do audio

def transcript_google():

    print("Starting transcript...")

    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=TS,
        language_code="pt-BR",
    )

    with open("output.wav", 'rb') as f:
        content = f.read()

    audio = { "content": content }

    # Detects speech in the audio file
    response = client.recognize(config=config, audio=audio)

    for result in response.results:
        if('feij' in result.alternatives[0].transcript):
            return result.alternatives[0].transcript

# Funnção para devolver o comando que deve ser passado pra máquina

def get_command(sentence):

    words = sentence.split(' ')
    stems = [stemmer.stem(word) for word in words]

    if('lig' in stems or 'acion' in stems or 'acend' in stems):
        return b'1'
    elif('deslig' in stems):
        return b'0'
    elif('pisc' in stems or 'pisq' in stems or 'pic' in stems or 'piq' in stems or 'pixel' in stems):
        return b'2'
    else:
        print(stems)
        return b'3'

# Função principal

if __name__ == "__main__":

    # Argumentos que devem ser passados

    if (len(sys.argv) == 1):
        print("ERROR: No COM argument.")
        exit(0)
    elif (len(sys.argv) == 2):
        print("ERROR: No Baudrate argument.")
        exit(0)
    
    com = sys.argv[1]
    baudrate = sys.argv[2]

    # Iniciar porta serial

    serial = serial.Serial(port = com, baudrate = baudrate)

    # Loop principal

    while True:
        
        print(f'Waiting for data on {com} w/ baudrate {baudrate}...')
        data = serial.readline().decode('ASCII').replace('\n', '')
        if (data == START_CHAR):
            audio = list()
            data = serial.readline().decode('ASCII').replace('\n', '')
            while (data != END_CHAR):
                audio.append(data)
                data = serial.readline().decode('ASCII').replace('\n', '')
            
            print(f'Received {len(audio)} entries')
            convert(audio)
            transcript = transcript_google()
            command = get_command(transcript)
            print(command)
            serial.write(command)

        time.sleep(5)