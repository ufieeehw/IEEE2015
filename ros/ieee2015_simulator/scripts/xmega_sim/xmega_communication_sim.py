import subprocess
import sys
import serial

# must install socat to operate
# www.dest-unreach.org/socat/
# or 
# sudo apt-get install socat


# subprocess to open fake COM Ports
subprocess.call(["chmod", "u+x", "./com_ports_on.sh"])
subprocess.Popen(["xterm","-hold", "-e", "./com_ports_on.sh"])

print "hello World"
