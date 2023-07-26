#!/usr/bin/env python

import rospy, os, sys, signal, time
from tf import transformations, TransformBroadcaster
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from multiprocessing import Process, Queue
from std_msgs.msg import String, Int16MultiArray
from math import sin, cos, pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

L  = .1185
W  = .0825
SL = (L*L+W*W)**.5
LS = 1/SL
r  = .0395
R  = 2*3.14159*r



encoder_topic = "/encoder_feedback"
encoder_node = "encoder_node"



def publishOdomData(odom_pub, Vx, Vy, W0, cur_robot_position):

	odom_quat = transformations.quaternion_from_euler(0, 0, cur_robot_position[2])
	odom_broadcaster = TransformBroadcaster()
	cur_time = rospy.Time.now()
	odom_broadcaster.sendTransform(
		(cur_robot_position[0], cur_robot_position[1], 0.),
		odom_quat,
		cur_time,
		"base_footprint",
		"odom"
	)

	odom = Odometry()
	odom.header.stamp = cur_time
	odom.header.frame_id = "odom"

	# set the position
	odom.pose.pose = Pose(Point(cur_robot_position[0], cur_robot_position[1], 0.), Quaternion(*odom_quat))

	# set the velocity
	odom.child_frame_id = "base_footprint"
	odom.twist.twist = Twist(Vector3(Vx, Vy, 0), Vector3(0, 0, W0))
	# publish the message
	odom_pub.publish(odom)

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
	ppsp= [ 5100,  5125,  5185, 5325]
	ppsn= [5065, 5230, 5270, 5175]
	W_speed = [(int(vals)*max_wheel_speed_pos[ind]/ppsp[ind]) if int(vals) > 0 else (int(vals)*max_wheel_speed_neg[ind]/ppsn[ind]) for ind, vals in enumerate(datas.data.strip().split(","))]
	# W_speed = [int(vals) if vals > 100 else 0 for vals in datas.data.strip().split(",")]
	queue.put(W_speed)
	# if (W_speed != [0, 0, 0, 0]):
	# 	print(W_speed)

def encoderFeedbackThread(queue):
	rospy.init_node(encoder_node, anonymous=True)
	rospy.Subscriber(encoder_topic, String, encoderFeedbackCB, callback_args=queue, queue_size=20)
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
	joint_msg.name = ["wheel_lf_joint", "wheel_rf_joint", "wheel_lb_joint", "wheel_rb_joint"]
	joint_msg.position = wheel_pos
	joint_msg.velocity = []
	joint_msg.effort = []
	joint_pub.publish(joint_msg)
	prev_pos = joint_msg.position

publishing_frequency = 100
finalSpeedArray = []



def wheelSpeedPublisher(queue):
	
	wheel_msg = Int16MultiArray(); wheel_msg.data = [0, 0, 0, 0]
	
	rospy.init_node('joystick_node', anonymous=True)
	pwm_pub = rospy.Publisher("/wheel_pwm", Int16MultiArray, queue_size=10)
	rate = rospy.Rate(publishing_frequency) 
	pwm_pub.publish(wheel_msg)
	
	currentSpeedArray = []

	forwards = False
	currentPwm = -255
	while not rospy.is_shutdown():
		wheel_msg.data = [currentPwm, currentPwm, currentPwm, currentPwm] # [W3, W4, W1, W2]
		pwm_pub.publish(wheel_msg)



		currentSpeed = queue.get()
		currentSpeedArray.append(currentSpeed)

		if len(currentSpeedArray) >=200:
			dataNpArray = np.array(currentSpeedArray[50:])
			dataAvg = np.average(dataNpArray, axis=0)
			daAvgList = dataAvg.tolist()
			prdatStr = "["+str(currentPwm)+" ,"+str(daAvgList[0])+", "+str(daAvgList[1])+","+str(daAvgList[2])+", "+str(daAvgList[3])+"],"
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
			break
			



if __name__ == "__main__":
	
	queue = Queue()
	queue.put([0, 0, 0, 0])
	encoder_process = Process(target=encoderFeedbackThread, name="omobotmovement", args=[queue])
	encoder_process.start()
	wheelSpeedPublisher(queue)


















def getPwmFromAngSpeed(x, mode = 0):
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