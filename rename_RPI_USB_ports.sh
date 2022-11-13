#!/bin/bash

# Author: sunfu-chou

# ------------------------------------
# | /dev/USB2 | /dev/USB0 |          |
# ------------------------- Ethernet |
# | /dev/USB3 | /dev/USB1 |          |
# ------------------------------------

echo  'KERNEL=="ttyUSB*", KERNELS=="1-1.1", MODE:="0777", SYMLINK+="USB0"' > /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyUSB*", KERNELS=="1-1.2", MODE:="0777", SYMLINK+="USB1"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyUSB*", KERNELS=="1-1.3", MODE:="0777", SYMLINK+="USB2"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyUSB*", KERNELS=="1-1.4", MODE:="0777", SYMLINK+="USB3"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyS*", MODE:="0777", GROUP:="dialout",  SYMLINK+="SERIAL0"' >> /etc/udev/rules.d/rpi-usb.rules

service udev reload
echo 'wait...'

sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service

sleep 2
service udev restart
cat /etc/udev/rules.d/rpi-usb.rules
echo 'finished.'