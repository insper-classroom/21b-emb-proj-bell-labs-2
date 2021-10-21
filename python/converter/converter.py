import numpy as np
import scipy.io.wavfile
import audiotools

DEFAULT_FREQUENCY = 11000

class Converter:
    def __init__(self):
        self.language = ''
    
    def convert_audio(self, audio):
        audio = np.array(audio, dtype = np.int)

        scipy.io.wavfile.write("tmp.wav", DEFAULT_FREQUENCY, audio)
        audiotools.open("tmp.wav").convert("tmp.flac", audiotools.FlacAudio, audiotools.DEFAULT_QUALITY)