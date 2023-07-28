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


def fittingFunc(x, a, b):
	return b + a/x

def ReverseFitFunc(x, a, b):
	return a / (x - b)

def processData(csvFilePath):
	xp = []
	xn = []
	yp = [[], [], [], []]
	yn = [[], [], [], []]

	csvDF = pd.read_csv(csvFilePath)
	values = csvDF.values

	for i in range(len(values)):
		x_temp = values[i][0]

		if x_temp < 0:
			xn.append(x_temp)
			yn[0].append(values[i][1])
			yn[1].append(values[i][2])
			yn[2].append(values[i][3])
			yn[3].append(values[i][4])

		else:
			xp.append(x_temp)
			yp[0].append(values[i][1])
			yp[1].append(values[i][2])
			yp[2].append(values[i][3])
			yp[3].append(values[i][4])

	return xp, yp, xn, yn


def getParameters(fittingFunc, pwmSpeed_p, encSpeed_p, pwmSpeed_n, encSpeed_n):
	ab_p = []
	var_p = []

	ab_n = []
	var_n = []
	for i in range(4):
		pop, pcp = curve_fit(fittingFunc, pwmSpeed_p, encSpeed_p[i])
		# pop, pcp = curve_fit(ReverseFitFunc, encSpeed_p[i], pwmSpeed_p)
		# saveCsv2(f'x_yp{i+1}.csv', encSpeed_p[i], pwmSpeed_p)
		ab_p.append(np.around(pop, 5))
		var_p.append(np.around(pcp, 5))

		pon, pcn = curve_fit(fittingFunc, pwmSpeed_n, encSpeed_n[i])
		# pon, pcn = curve_fit(ReverseFitFunc, encSpeed_n[i], pwmSpeed_n)
		# saveCsv2(f'x_yn{i+1}.csv', encSpeed_n[i], pwmSpeed_n)
		ab_n.append(np.around(pon, 5))
		var_n.append(np.around(pcn, 5))

	return ab_p, var_p, ab_n, var_n


def getFittingParameters(plot_=True):
	pwmSpeed_p, encSpeed_p, pwmSpeed_n, encSpeed_n = processData(csvFilePath)
	# fitFunc = ReverseFitFunc
	ab_p, var_p, ab_n, var_n = getParameters(fittingFunc, pwmSpeed_p, encSpeed_p, pwmSpeed_n, encSpeed_n)


max_wheel_rpm = [36, 36, 36, 36]
max_wheel_rpm_pos = [35.6, 35.9, 37.25, 36.15]
max_wheel_rpm_neg = [35.3, 36.7, 36.18, 36.65]
max_wheel_speed_pos = [vals*2*3.1416/60 for vals in max_wheel_rpm_pos]
max_wheel_speed_neg = [vals*2*3.1416/60 for vals in max_wheel_rpm_neg]


def encoderFeedbackCB(datas, queue):
	ppsp = [5100,  5125,  5185, 5325]
	ppsn = [5065, 5230, 5270, 5175]
	# W_speed = [(int(vals)*max_wheel_speed_pos[ind]/ppsp[ind]) if int(vals) > 0 else (int(vals)*max_wheel_speed_neg[ind]/ppsn[ind]) for ind, vals in enumerate(datas.data.strip().split(","))]
	W_speed = [round(int(vals)*max_wheel_speed_pos[ind]/ppsp[ind], 5) if int(vals) > 0 else round(int(vals)
																								  * max_wheel_speed_neg[ind]/ppsn[ind], 5) for ind, vals in enumerate(datas.data.strip().split(","))]
	# W_speed = [int(vals) if vals > 100 else 0 for vals in datas.data.strip().split(",")]
	queue.put(W_speed)
	# if (W_speed != [0, 0, 0, 0]):
	# 	print(W_speed)


def encoderFeedbackThread(queue):
	rospy.init_node(encoder_node, anonymous=True)
	rospy.Subscriber(encoder_topic, String, encoderFeedbackCB,
					 callback_args=queue, queue_size=20)
	signal.signal(signal.SIGINT, signal_handler)
	rospy.spin()


def angularToCmdVel(W):
	Vx = (W[0] + W[1] + W[2] + W[3]) * r / 4
	Vy = (-W[0] + W[1] + W[2] - W[3]) * r / 4
	W0 = (-W[0]*LS + W[1]*LS - W[2]*LS + W[3]*LS) * r / 4

	return Vx, Vy, W0


def get_cur_robot_pos(Vx, Vy, W0, cur_pos):
	global prevTimePos

	curTime = rospy.Time.now().to_sec()
	dT = (curTime-prevTimePos)
	prevTimePos = curTime

	dW = W0 * dT
	W0 = cur_pos[2] + dW

	dx = (Vx * cos(W0) - Vy * sin(W0)) * dT
	dy = (Vx * sin(W0) + Vy * cos(W0)) * dT

	# if dx!= 0.0 or dy!=0.0 or dW!=0.0:
	# 	print([dx, dy, dW], dT)
	return [cur_pos[0]+dx, cur_pos[1]+dy, W0]


def get_cur_wheel_pos(W, cur_pos):
	global prevTimeWhl
	curTime = rospy.Time.now().to_sec()
	passedTime = (curTime-prevTimeWhl)
	prevTimeWhl = curTime
	# print([W[0], W[1], W[2], W[3]], end="\t")
	return [cur_pos[0]+W[0]*passedTime, cur_pos[1]+W[1]*passedTime, cur_pos[2]+W[2]*passedTime, cur_pos[3]+W[3]*passedTime]


def publishJointData(joint_pub, wheel_pos):
	joint_msg = JointState()
	joint_msg.header = Header()
	joint_msg.header.stamp = rospy.Time.now()
	joint_msg.name = ["wheel_lf_joint", "wheel_rf_joint",
					  "wheel_lb_joint", "wheel_rb_joint"]
	joint_msg.position = wheel_pos
	joint_msg.velocity = []
	joint_msg.effort = []
	joint_pub.publish(joint_msg)
	prev_pos = joint_msg.position


publishing_frequency = 100
finalSpeedArray = []


def wheelSpeedPublisher(queue):

	wheel_msg = Int16MultiArray()
	wheel_msg.data = [0, 0, 0, 0]

	rospy.init_node('joystick_node', anonymous=True)
	pwm_pub = rospy.Publisher("/wheel_pwm", Int16MultiArray, queue_size=10)
	rate = rospy.Rate(publishing_frequency)
	pwm_pub.publish(wheel_msg)

	currentSpeedArray = []

	forwards = False
	currentPwm = -255
	while not rospy.is_shutdown():
		wheel_msg.data = [currentPwm, currentPwm,
						  currentPwm, currentPwm]  # [W3, W4, W1, W2]
		pwm_pub.publish(wheel_msg)

		currentSpeed = queue.get()
		currentSpeedArray.append(currentSpeed)
		csvData = ""
		if len(currentSpeedArray) >= 200:
			dataNpArray = np.array(currentSpeedArray[50:])
			dataAvg = np.average(dataNpArray, axis=0)
			daAvgList = dataAvg.tolist()
			prdatStr = str(currentPwm)+" ,"+str(daAvgList[0])+", "+str(
				daAvgList[1])+","+str(daAvgList[2])+", "+str(daAvgList[3])
			csvData += prdatStr+"\n"
			print(prdatStr)
			currentSpeedArray = []
			currentPwm += 5
			wheel_msg.data = [0, 0, 0, 0]
			while queue.get() != [0, 0, 0, 0]:
				pwm_pub.publish(wheel_msg)

			# time.sleep(3)
			rate.sleep()

		rate.sleep()

		if not forwards and currentPwm > -50:
			currentPwm = 50
			forwards = True
		if forwards and currentPwm > 255:
			with open(csvFilePath, 'w') as wf:
				wf.write(csvData)
			getFittingParameters()
			break


if __name__ == "__main__":

	queue = Queue()
	queue.put([0, 0, 0, 0])
	encoder_process = Process(
		target=encoderFeedbackThread, name="omobotmovement", args=[queue])
	encoder_process.start()
	wheelSpeedPublisher(queue)


def getPwmFromAngSpeed(x, mode=0):
	yp = [0, 0, 0, 0]
	yn = [0, 0, 0, 0]
	ap = [-143.48526, -152.71374, -157.26158, -156.59191]
	bp = [4.20121, 4.32203, 4.28761, 4.37398]
	an = [-144.26665, -158.83706, -156.74901, -150.89588]
	bn = [-4.2302, -4.37313, -4.41995, -4.28225]
	if mode:
		yp[0] = -143.48526/x[0] + (4.20121)
		yn[0] = -144.26665/x[0] + (-4.2302)
		yp[1] = -152.71374/x[1] + (4.32203)
		yn[1] = -158.83706/x[1] + (-4.37313)
		yp[2] = -157.26158/x[2] + (4.28761)
		yn[2] = -156.74901/x[2] + (-4.41995)
		yp[3] = -156.59191/x[3] + (4.37398)
		yn[3] = -150.89588/x[3] + (-4.28225)

	else:
		yp[0] = -143.48526 / (x[0] - (4.20121))
		yn[0] = -144.26665 / (x[0] - (-4.2302))
		yp[1] = -152.71374 / (x[1] - (4.32203))
		yn[1] = -158.83706 / (x[1] - (-4.37313))
		yp[2] = -157.26158 / (x[2] - (4.28761))
		yn[2] = -156.74901 / (x[2] - (-4.41995))
		yp[3] = -156.59191 / (x[3] - (4.37398))
		yn[3] = -150.89588 / (x[3] - (-4.28225))

	if x > 0:
		return yp
	else:
		return yn
