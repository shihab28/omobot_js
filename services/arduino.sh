#!/bin/sh
echo ${PASS_} | sudo chmod 777 -s /dev/ttyACM0 
echo ${PASS_} | sudo chmod 777 -s /dev/ttyACM1
cd ~/Downloads/Arduino && ./arduino-ide &