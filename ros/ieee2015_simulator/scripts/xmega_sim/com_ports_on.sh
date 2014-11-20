#!/bin/bash
echo "Initializing socat COM simulation..."
echo "xMega ports created"
echo 
echo
echo "Ports will be closed when this window is closed"

socat pty,link=$HOME/COM1 pty,link=$HOME/COM2

