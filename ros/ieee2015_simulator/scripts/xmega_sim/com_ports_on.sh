#!/bin/bash
echo "Initializing socat COM simulation..."
echo "xMega ports created"
echo 
echo
echo "Ports will be closed when this window is closed"

socat -d -d pty,raw,echo=1 pty,raw,echo=1
