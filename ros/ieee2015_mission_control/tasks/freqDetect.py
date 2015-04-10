import wave
import numpy as np
import struct
import math

# takes wav file input, outputs the sequence of colors from the simon says
def detectColorAudioSequence(audFile):
    # Input file must be in mono
    
    # the color plays for about 0.25 sec followed by appx 0.15 sec of silence before the next color
    dataDiv = 0.1 # size of division in seconds
    
    yellowFreq = 780 # frequencies of sounds for each color
    redFreq = 976
    blueFreq = 1080
    greenFreq = 1268
    
    freqSpan = 10 # acceptable range around color frequency (plus or minus)
    prevFreqSpan = 10 # acceptable range to save a new frequency (plus or minus)


    wavFile = wave.open(audFile, 'r') # open file in read only
    dataSize = (wavFile.getnframes())
    fileRate = wavFile.getframerate()

    data = wavFile.readframes(dataSize) # reads 'dataSize' number of bytes
    wavFile.close()
    data = struct.unpack('{dS}h'.format(dS = dataSize), data) # unpacks bytes into 'dataSize' number of shorts
    data = np.array(data) # puts data in numpy array

    # the entire input file is divided into 0.1 sec chunks
    chunkSize = dataDiv*fileRate
    chunkQuantity = int(math.ceil(dataSize/chunkSize)) # number of chunks

    freqArray = np.zeros(shape=(chunkQuantity,)) # array to hold frequencies
    freqArraySize = 0
    prevFreq = 0
    
    for i in range(chunkQuantity): # for each chunk
        dataChunk = data[i*chunkSize:(i+1)*chunkSize:] # get chunk from data array
        chunkFreq = detectFreq(dataChunk, fileRate) # detect frequency of chunk
        
        if(((prevFreq - prevFreqSpan) > chunkFreq) or (chunkFreq > (prevFreq + prevFreqSpan))):
            # if the detected chunk frequency is outside of a set range from the previous frequency
            # i.e. a new frequency is detected, add to frequency array
            freqArray[freqArraySize] = chunkFreq
            freqArraySize += 1
        
        prevFreq = chunkFreq

   
   
    for i in range(freqArraySize):
        x = freqArray[i]
        # checks if frequencies are in the range of the various color frequencies
        if ((yellowFreq - freqSpan <= x) & (x <= yellowFreq + freqSpan)):
            print "Yellow"
        elif ((redFreq - freqSpan <= x) & (x <= redFreq + freqSpan)):
            print "Red"
        elif ((blueFreq - freqSpan <= x) & (x <= blueFreq + freqSpan)):
            print "Blue"
        elif ((greenFreq - freqSpan <= x) & (x <= greenFreq + freqSpan)):
            print "Green"


def detectFreq(dataArray, fileRate):
    
    discreteTransform = np.fft.fft(dataArray) # fast fourier transform
    freqs = np.fft.fftfreq(len(discreteTransform)) # get frequencies
    maxFreq = np.argmax(np.abs(discreteTransform)) # get maximum frequency index
    freq = freqs[maxFreq] # get maximum frequency
    hertz = abs(freq * fileRate) # get frequency in hertz
    
    return hertz
    


path = "./simon_audio.wav"
detectColorAudioSequence(path)


