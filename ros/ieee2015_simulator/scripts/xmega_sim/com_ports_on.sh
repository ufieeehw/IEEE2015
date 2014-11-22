#!/bin/bash
echo "Initializing socat COM simulation..."
echo "xMega ports created"
echo 
echo
echo "Ports will be closed when this window is closed"

socat -d -d pty,link=/dev/ttyS30,echo=1 pty,link=/dev/xmega_tty,echo=1 


 


