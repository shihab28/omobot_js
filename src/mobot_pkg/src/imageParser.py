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
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os
from filelock import FileLock

# Define the memory path for storing images. Using shared memory for efficiency.
mempath = "/dev/shm/"
# Ensure the directory exists, create if not.
if not os.path.isdir(mempath):
    os.makedirs(mempath)

class ImageSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_subscriber', anonymous=True)
        # Subscribe to the image topic
        self.image_subscriber = rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1)
        # Initialize the CvBridge
        self.cv_bridge = CvBridge()
        # Placeholder for the latest image
        self.image = None
        # Lock for thread-safe writing to disk
        self.lock = FileLock(mempath + "file.lock")
        # Image ID for creating unique filenames
        self.imgId = 0

    def image_callback(self, msg):
        # Callback function for image topic subscription
        try:
            # Convert the ROS image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image = cv_image
            # Use the file lock to ensure thread-safe writing
            with self.lock:
                # Save the image to the specified location with a unique filename
                cv2.imwrite(mempath + "testImage{}.png".format(self.imgId), cv_image)
                # Increment image ID for the next image
                self.imgId += 1
        except Exception as e:
            self.image = None
            rospy.logerr("Error processing image: %s", str(e))
        
    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    image_subscriber.run()