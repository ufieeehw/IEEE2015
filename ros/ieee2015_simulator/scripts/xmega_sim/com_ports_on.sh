#!/bin/bash

socat -d pty,link=/dev/ttyS30,echo=1 pty,link=/dev/xmega_tty,echo=1 