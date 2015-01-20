INSTRUCTIONS FOR LAUNCHING AND OPERATING XMEGA SIMULATOR

There are a few programs and libraries that must be installed first

Socat:

	1. www.dest-unreach.org/socat/ or 
	2. sudo apt-get install socat

PyUnit:

	1. http://sourceforge.net/projects/pyunit/?source=typ_redirect 
	2. Navigate to unzipped tar folder and run 'python setup.py install'

PySerial:
	
	1. https://pypi.python.org/pypi/pyserial

For proper use must disable password requirement when running with sudo 
for file 'xmega_communication_sim.py'

HOW TO:

	1. sudo visudo -f /etc/sudoers.d/90-cloudimg-ubuntu
	
	2. youruserame ALL=(ALL) NOPASSWD: local/path/to/file/python xmega_communication_sim.py
	   or,
	   youruserame ALL=(ALL) NOPASSWD: ALL --> Not safe but works as last resort

To launch full simulation run the bash script titled 'main_start.sh'

Running 'main_start.sh' will pop up three new terminal windows. 

	1. XMEGA SIMULATOR 

	2. ROS SIMULATOR

	3. xmega_driver Launch File
	
All these subprocesses can be stopped with ctrl-C and restarted from the same terminal window
Works as a way to pause and start once originally open

TO DO:

	1. Impliment as Unit Test 

	2. Add functions to ROS_send as neew ROS topcs are added to xmega_driver
