INSTRUCTIONS FOR LAUNCHING AND OPERATING XMEGA SIMULATOR
 
Must install socat to operate
www.dest-unreach.org/socat/
or 
sudo apt-get install socat

------------------------------------- READ  ME -------------------------------------------------------

To launch full simulation run the bash script titled 'main_start.sh' 
- DO NOT RUN WITH SUDO --> For some reason when it is run as admin the ROS side won't work

Running 'main_start.sh' will pop up two new terminal windows. 

	Step 1. XMEGA SIMULATOR Window --> Enter root password to launch Xmega Sim

	Step 2. ROS SIMULATOR window --> Turn on or leave off

	Step 3. Once XMEGA SIMULATOR is on, return to orignal terminal window to auto launch 
			the xmega_driver launch file

	Step 4. Watch the magic happen

-------------------------------------------------------------------------------------------------------	

TO DO:
	1. Automate entire process --> Cannot do as long as running 'main_start.sh' as root 
								crashes 'ROS_send.py' 
								or
								Figure out a way to access tty ports when not running as root

								One of those solutions can automate process

	2. Parse Types in 'ROS_send.py' to send all possible codes through xmega_driver

	3. Impliment as Unit Test --> requires fulll automation
