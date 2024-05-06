#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import shutil, os
from filelock import FileLock

# Define the memory path for storing images. Using shared memory for efficiency.
mempath = "/dev/shm/"

# Ensure the directory exists, create if not.

print (mempath, " ; ", os.listdir(mempath))
# try:
#     if not os.listdir(mempath) == []:
#         shutil.rmtree(mempath, ignore_errors=True)
# except:
#     pass
if not os.path.isdir(mempath):
    os.makedirs(mempath)


lock = FileLock(mempath + "file.lock")

def scan_callback(msg):
    
    start_index = -len(msg.ranges)//8
    end_index = len(msg.ranges)//8
    # print(start_index, end_index, len(msg.ranges))
    ranges_in_fov = msg.ranges[:len(msg.ranges)//8:1]
    # print(ranges_in_fov)
    with lock:
        np.save(mempath + "testScans{}.npy".format(0), np.array(ranges_in_fov))

    

def listener():
    rospy.init_node('laser_scan_processor', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    listener()
