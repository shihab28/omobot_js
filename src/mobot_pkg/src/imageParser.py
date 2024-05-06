#!/usr/bin/env /usr/bin/python2

'''
Author Information
Author: Shihab Uddin Ahamad
Email: shihab.ahamad28@gmail.com
Date: 03/20/2024
Description: 
    This ROS script subscribes to the "/camera/image_raw" topic to receive image messages. Upon receiving an image, it converts the ROS image message to an OpenCV format using CvBridge. Then, it saves the image to a specific location `mempath`, specifically "/dev/shm/" to utilize shared memory for faster access and reduced disk wear. The script uses file locking to ensure thread-safe writing to the disk, making it suitable for applications where multiple processes might attempt to write to the same file. It is intended to capture and process images from the a ROS image topic, and use the saved image for fall detection later. 
'''

# Import necessary libraries
import rospy,  cv2, os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from filelock import FileLock
from sensor_msgs.msg import LaserScan
import numpy as np
import shutil

# Define the memory path for storing images. Using shared memory for efficiency.
mempath = "/dev/shm/"

# Ensure the directory exists, create if not.

print (mempath, " ; ", os.listdir(mempath))
try:
    if not os.listdir(mempath) == []:
        shutil.rmtree(mempath, ignore_errors=True)
except:
    pass
if not os.path.isdir(mempath):
    os.makedirs(mempath)

class ImageSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_subscriber', anonymous=False)
        # Subscribe to the image topic
        # self.image_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        
        self.image_subscriber = rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1)
        
        # Initialize the CvBridge
        self.cv_bridge = CvBridge()
        # Placeholder for the latest image
        self.image = None
        self.scan_array = np.array([])
        # Lock for thread-safe writing to disk
        self.lock = FileLock(mempath + "file.lock")
        # Image ID for creating unique filenames
        self.imgId = ""
        self.fov_start_angle = -45.0 * (3.14159265 / 180.0) 
        self.fov_end_angle = 45.0 * (3.14159265 / 180.0)
        

    def scan_callback(self, msg):
        msgArray = msg.ranges
        
        start_index = -len(msgArray)//8
        end_index = len(msgArray)//8
        # print(start_index, end_index, len(msgArray))
        ranges_in_fov = msgArray[start_index:end_index:1]
        
        self.scan_array = np.array(ranges_in_fov)
        

    def image_callback(self, msg):
        # Callback function for image topic subscription
        try:
            # Convert the ROS image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image = cv_image
            # Use the file lock to ensure thread-safe writing
            with self.lock:
                # Save the image to the specified location with a unique filename
                
                # rospy.logerr("can_array: %s", ",".join([str(vals) for vals in self.scan_array]))
                cv2.imwrite(mempath + "testImage{}.png".format(self.imgId), cv_image)
                # Increment image ID for the next image
                # self.imgId += 1
        except Exception as e:
            print(e)
            self.image = None
            rospy.logerr("Error processing image: %s", str(e))
        
    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    image_subscriber.run()