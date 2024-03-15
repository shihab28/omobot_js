#!/usr/bin/env python

import rospy, os, sys, signal
from tf import transformations, TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from multiprocessing import Process, Queue
from std_msgs.msg import String
from math import sin, cos, pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from fiducial_msgs.msg import FiducialTransformArray, FiducialArray

L  = .1185
W  = .0825
SL = (L+W)
LS = 1/SL
r  = .0398
R  = 2*3.14159*r



marker_topic = "/fiducial_vertices"
marker_node = "marker_detect"



def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	os.system('echo y | rosnode cleanup')
	sys.exit(0)


def markerFeedbackCB(datas, queue):
	print("markerFeedbackCB : ", datas)

def markerFeedbackThread(queue):
	rospy.init_node(marker_node, anonymous=True)
 
	rospy.Subscriber(marker_topic, FiducialArray, markerFeedbackCB, callback_args=queue, queue_size=10)
	signal.signal(signal.SIGINT, signal_handler)	
	rospy.spin()



if __name__ == "__main__":
	
	queue = Queue()
	queue.put([0, 0, 0, 0])
	
	markerFeedbackThread(queue)