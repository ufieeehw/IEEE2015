import wave
import numpy as np
import struct
import math


def detectColorAudioSequence(audFile):
    # Input file must be in mono
    
    dataDiv = 0.1 # size of division in seconds
    
    yellowFreq = 780
    redFreq = 976
    blueFreq = 1080
    greenFreq = 1268
    
    freqSpan = 10 # acceptable range around color frequency
    prevFreqSpan = 10 # acceptable range to save a new frequency


    wavFile = wave.open(audFile, 'r') # open file in read only
    dataSize = (wavFile.getnframes() - 1)
    fileRate = wavFile.getframerate()

    data = wavFile.readframes(dataSize) # reads 'dataSize' number of bytes
    wavFile.close()
    data = struct.unpack('{dS}h'.format(dS = dataSize), data) # unpacks bytes into 'dataSize' shorts
    data = np.array(data) # puts data in numpy array

    chunkSize = dataDiv*fileRate
    chunkQuantity = int(math.ceil(dataSize/chunkSize))

    freqArray = np.zeros(shape=(chunkQuantity,))
    freqArraySize = 0
    prevFreq = 0
    
    for i in range(chunkQuantity):
        dataChunk = data[i*chunkSize:(i+1)*chunkSize:]
        chunkFreq = detectFreq(dataChunk, fileRate)
        
        if(((prevFreq - prevFreqSpan) > chunkFreq) or (chunkFreq > (prevFreq + prevFreqSpan))):
            freqArray[freqArraySize] = chunkFreq
            freqArraySize += 1
        
        prevFreq = chunkFreq

   
   
    for i in range(freqArraySize):
        x = freqArray[i]

        if ((yellowFreq - freqSpan <= x) & (x <= yellowFreq + freqSpan)):
            print "Yellow"
            #print x
            #print '\n'
        elif ((redFreq - freqSpan <= x) & (x <= redFreq + freqSpan)):
            print "Red"
            #print x
            #print '\n'
        elif ((blueFreq - freqSpan <= x) & (x <= blueFreq + freqSpan)):
            print "Blue"
            #print x
            #print '\n'
        elif ((greenFreq - freqSpan <= x) & (x <= greenFreq + freqSpan)):
            print "Green"
            #print x
            #print '\n'        



def detectFreq(dataArray, fileRate):
    
    discreteTransform = np.fft.fft(dataArray) # fast fourier transform
    freqs = np.fft.fftfreq(len(discreteTransform)) # get frequencies
    maxFreq = np.argmax(np.abs(discreteTransform)) # get maximum frequency index
    freq = freqs[maxFreq] # get maximum frequency
    hertz = abs(freq * fileRate) # get frequency in hertz
    
    return hertz
    




path = "./simon_audio.wav"
detectColorAudioSequence(path)


