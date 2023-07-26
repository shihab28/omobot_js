#!/bin/bash

# ls /dev/input/
# sudo jstest /dev/input/js0
# ls -l /dev/input/js0
# sudo chmod a+rw /dev/input/js0
rosparam set joy_node/dev "/dev/input/js0"
rosrun joy joy_node #
# rostopic echo joy #new terminal