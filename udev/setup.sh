#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# ln -s $DIR/99-ftdi.rules /etc/udev/rules.d/99-ftdi.rules
cp $DIR/99-ftdi.rules /etc/udev/rules.d/99-ftdi.rules
udevadm control --reload-rules
udevadm trigger
