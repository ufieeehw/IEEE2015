record_wav.py Is designed to record the audio for a time corrseponding to the sequence level simon says is on (starting at 1),
then saves it as a .wav file. Currently it decided the time by multiplying the level by (the length of the sound + time before the next sound) + half a second. It saves a file in the same directory for freqDetect.py to read. Alsaaudio must be installed! Use "apt-get install python-alsaaudi" to do so. 
All it needs instructions for what level to record for.

reqDetect.py Is designed to find the frequency of the recorded audio and decide the color it corresponds too. Currently it prints out the color's name in the order it happens. It uses the simon_audio.wav (which is recorded in mono), so must be run after record_wav.py.
