#!/usr/bin/env /usr/bin/python2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os
from filelock import FileLock

mempath = "/dev/shm/"
if os.path.isdir("/dev/shm/") == False:
    os.makedirs(mempath)
# lock = FileLock(mempath + "testImage.lock")    
# fileNames = [
#    "testImage0.png",
#    "testImage1.png",
#    "testImage2.png",
#    "testImage3.png",
#]

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.image_subscriber = rospy.Subscriber(
            '/camera/image_raw', Image, self.image_callback, queue_size=1)
        self.cv_bridge = CvBridge()
        self.image = None
        self.completedDetection = True
        self.lock = FileLock(mempath + "file.lock")
        self.imgId = 0

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            self.image = cv_image
            with self.lock:
                cv2.imwrite(mempath + "testImage{}.png".format(self.imgId), cv_image)
 
        except Exception as e:
            self.image = None
            rospy.logerr("Error processing image: %s", str(e))
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    image_subscriber.run()
