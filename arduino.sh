#!/bin/sh

echo 1628 | sudo chmod 777 -s /dev/ttyACM0 
echo 1628 | sudo chmod 777 -s /dev/ttyACM1
cd ~/Downloads/Arduino && ./arduino-ide &