#!/bin/sh -e

echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="RPLIDAR"' | sudo tee /etc/udev/rules.d/rplidar.rules
sudo udevadm control --reload-rules

echo "Udev rules created and reloaded!"
