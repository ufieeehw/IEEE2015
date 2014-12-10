#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cp $DIR/98-ftdi.rules /etc/udev/rules.d/98-ftdi.rules
udevadm control --reload-rules
udevadm trigger
