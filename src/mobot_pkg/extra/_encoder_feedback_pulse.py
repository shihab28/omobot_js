#!/usr/bin/env python

import rospy
import os
import sys
import signal
import time
from tf import transformations, TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from multiprocessing import Process, Queue
from std_msgs.msg import String, Int16MultiArray
from math import sin, cos, pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
from scipy.optimize import curve_fit
import pandas as pd
import matplotlib.pyplot as plt

L = .1185
W = .0825
SL = (L+W)
LS = 1/SL
r = .0398
R = 2*3.14159*r


encoder_topic = "/encoder_feedback"
encoder_node = "encoder_node"
csvFilePath = 'src/mobot_pkg/extra/pwmX_vs_W.csv'


def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	os.system('echo y | rosnode cleanup')
	sys.exit(0)


max_wheel_rpm = [36, 36, 36, 36]
max_wheel_rpm_pos = [35.6, 35.9, 37.25, 36.15]
max_wheel_rpm_neg = [35.3, 36.7, 36.18, 36.65]
max_wheel_speed_pos = [vals*2*3.1416/60 for vals in max_wheel_rpm_pos]
max_wheel_speed_neg = [vals*2*3.1416/60 for vals in max_wheel_rpm_neg]


def encoderFeedbackCB(datas, queue):
	ppsp = [5100,  5125,  5185, 5325]
	ppsn = [5065, 5230, 5270, 5175]
	# W_speed = [(int(vals)*max_wheel_speed_pos[ind]/ppsp[ind]) if int(vals) > 0 else (int(vals)*max_wheel_speed_neg[ind]/ppsn[ind]) for ind, vals in enumerate(datas.data.strip().split(","))]
	# W_speed = [round(int(vals)*max_wheel_speed_pos[ind]/ppsp[ind], 5) if int(vals) > 0 else round(int(vals) * max_wheel_speed_neg[ind]/ppsn[ind], 5) for ind, vals in enumerate(datas.data.strip().split(","))]
	W_speed = [int(vals) if vals > 100 else 0 for vals in datas.data.strip().split(",")]
	queue.put(W_speed)
	# if (W_speed != [0, 0, 0, 0]):
	# 	print(W_speed)


def encoderFeedbackThread(queue):
	rospy.init_node(encoder_node, anonymous=True)
	rospy.Subscriber(encoder_topic, String, encoderFeedbackCB,
					 callback_args=queue, queue_size=10)
	signal.signal(signal.SIGINT, signal_handler)
	rospy.spin()



publishing_frequency = 50
finalSpeedArray = []


def wheelSpeedPublisher(queue):

	wheel_msg = Int16MultiArray()
	wheel_msg.data = [0, 0, 0, 0]

	rospy.init_node('joystick_node', anonymous=True)
	pwm_pub = rospy.Publisher("/wheel_pwm", Int16MultiArray, queue_size=2)
	rate = rospy.Rate(publishing_frequency)
	pwm_pub.publish(wheel_msg)

	currentSpeedArray = []

	forwards = False
	currentPwm = 250
	cnt = 1
	while not rospy.is_shutdown():
		wheel_msg.data = [currentPwm, currentPwm,
						  currentPwm, currentPwm]  # [W3, W4, W1, W2]
		pwm_pub.publish(wheel_msg)

		currentSpeed = queue.get()
		currentSpeedArray.append(currentSpeed)
		csvData = ""
		if len(currentSpeedArray) >= 1100:
			dataNpArray = np.array(currentSpeedArray[100:])
			dataAvg = np.average(dataNpArray, axis=0)
			daAvgList = dataAvg.tolist()
			print( cnt, currentPwm,  rospy.Time.now().to_sec(),daAvgList)
			cnt += 1
			currentSpeedArray = []
			currentPwm = -250


		rate.sleep()


if __name__ == "__main__":

	queue = Queue()
	queue.put([0, 0, 0, 0])
	encoder_process = Process(
		target=encoderFeedbackThread, name="omobotmovement", args=[queue])
	encoder_process.start()
	wheelSpeedPublisher(queue)


dataPos = (1, 250, 1691594031.217158, [5045, 5023, 5193, 5055])
dataPNeg = (2, -250, 1691594057.601039, [-5037, -5197, -5099, -5211])