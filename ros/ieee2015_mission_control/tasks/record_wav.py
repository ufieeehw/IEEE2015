import alsaaudio, wave, numpy, time

'''apt-get install python-alsaaudio'''

def record_simon(num_of_sequence):
    inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE)
    inp.setchannels(1)
    inp.setrate(44100)
    inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
    inp.setperiodsize(1024)

    w = wave.open('simon_audio.wav', 'w')
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(44100)

    extra_time = num_of_sequence * (.25 + .15) + .5
    # Level of the sequence * (time for sounds to be made + time inbwtween sounds) + time inbetween sequences 

    timeout = time.time() + extra_time
    # Adding time out

    while True:
        l, data = inp.read()
        a = numpy.fromstring(data, dtype='int16')
        w.writeframes(data)
        if time.time() > timeout: 
        # Will break after safe time
            break

if __name__ == '__main__':
    # Need to pass in current level, starting at 1
    num_of_sequence = 1 # Just a place holder
    record_simon(num_of_sequence)
