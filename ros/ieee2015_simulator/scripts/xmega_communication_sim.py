import subprocess
import sys

# The main goal behind this application will be to create a bash script that 
# continously reads info from the Xmega Driver messages and outputs them to the terminal
# But before I output the data I am going to format ist accoring to Josh's standards
# I am going to to talk to Josh about his protocals so I can read info back from the terminal
# and format the messages into what they would actualy do on the xmega

while 1:

	#subprocess.check_call(["rostopic", "ieee2015_xmega_driver", "xmega_driver"])
	subprocess.check_call(["echo", "Hello Worl"])
	subprocess.check_call(["^D"])
	data = sys.stdin.readlines()
	print "Counted", len(data), "lines." 
