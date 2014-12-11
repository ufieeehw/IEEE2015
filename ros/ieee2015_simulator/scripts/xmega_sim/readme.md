INSTRUCTIONS FOR LAUNCHING AND OPERATING XMEGA SIMULATOR
 
Must install socat to operate
www.dest-unreach.org/socat/
or 
sudo apt-get install socat

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

	1. Parse Types in 'ROS_send.py' to send all possible codes through xmega_driver

	2. Impliment as Unit Test
