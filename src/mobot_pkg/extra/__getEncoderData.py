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

L  = .1185
W  = .0825
SL = (L+W)
LS = 1/SL
r  = .0398
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
	# odom.pose.pose = Pose(Point(cur_robot_position[0], cur_robot_position[1], 0.), Quaternion(*odom_quat))

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
max_wheel_rpm_pos = [35.6, 35.9, 37.25, 36.75]
max_wheel_rpm_neg = [-35.3, -36.1, -35.78, -35.05]
max_wheel_speed_pos = [vals*2*3.1416/60 for vals in max_wheel_rpm_pos]
max_wheel_speed_neg = [vals*2*3.1416/60 for vals in max_wheel_rpm_neg]

def encoderFeedbackCB(datas, queue):
	ppsp= [ 5100,  5125,  5325,  5185]
	ppsn= [-5065, -5230, -5175, -5270]
	# W_speed_ = [round(int(vals)*3.7699/ppsp[ind], 5) if int(vals) > 0 else round(int(vals)*3.7699/ppsn[ind], 5) for ind, vals in enumerate(datas.data.strip().split(","))]
	W_speed_ = [round(int(vals)*max_wheel_speed_pos[ind]/ppsp[ind], 5) if int(vals) > 0 else round(int(vals) * max_wheel_speed_neg[ind]/ppsn[ind], 5) for ind, vals in enumerate(datas.data.strip().split(","))]
	W_speed = [0.0 if abs(vals) < 0.0001 else vals for vals in W_speed_]
	queue.put(W_speed)
	# if (W_speed != [0, 0, 0, 0]):
	# 	print(W_speed)

def encoderFeedbackThread(queue):
	rospy.init_node(encoder_node, anonymous=True)
	rospy.Subscriber(encoder_topic, String, encoderFeedbackCB, callback_args=queue, queue_size=20)
	signal.signal(signal.SIGINT, signal_handler)	
	rospy.spin()

def angularToCmdVel(W):
	Vx = round((W[0] + W[1] + W[2] + W[3]) * r / 4, 4)
	Vy = round((-W[0] + W[1] + W[2] - W[3]) * r / 4, 4)
	W0 = round((-W[0]*LS + W[1]*LS - W[2]*LS + W[3]*LS) * r / 4, 4)

	return Vx, Vy, W0

def get_cur_robot_pos(Vx, Vy, W0, cur_pos):
	global prevTimePos

	curTime = rospy.Time.now().to_sec()
	dT = (curTime-prevTimePos)
	prevTimePos = curTime
	W_ang =  cur_pos[2]
	dW = W0 * dT

	dx = (Vx * cos(W_ang) - Vy * sin(W_ang)) * dT
	dy = (Vx * sin(W_ang) + Vy * cos(W_ang)) * dT
	
	# if abs(dx) >= 0.0003 or abs(dy)>=0.0003 or abs(dW) >=0.0001:
	# 	print([Vx, Vy, W0], "     ", [dx, dy, dW], "     ", dT)

	return [cur_pos[0]+dx, cur_pos[1]+dy, cur_pos[2] + dW]

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

refresh_rate = 50
def robotOdomPublisher(queue):
	global prevTimeWhl, prevTimePos
	curW = [0, 0, 0, 0]
	cur_robot_position = [0, 0, 0]
	cur_wheel_pos = [0, 0, 0, 0]
	rospy.init_node('joint_state_publisher', anonymous=True)
	odom_pub = rospy.Publisher('/odom', Odometry, queue_size=20)
	joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=20)

	prevTimePos = rospy.Time.now().to_sec()
	prevTimeWhl = rospy.Time.now().to_sec()
	rate = rospy.Rate(refresh_rate)
	rate.sleep()

	while not rospy.is_shutdown():
		# prevTimePos = rospy.Time.now().to_sec()
		# prevTimeWhl = rospy.Time.now().to_sec()
		try:
			curW = queue.get(timeout=1/refresh_rate)
		except:
			pass
		Vx, Vy, W0 = angularToCmdVel(curW)
		cur_robot_position = get_cur_robot_pos(Vx, Vy, W0, cur_robot_position)
		publishOdomData(odom_pub, Vx, Vy, W0, cur_robot_position)
		
		if (curW != [0, 0, 0, 0]):
			# print(rospy.Time.now().to_sec(), [Vx, Vy, W0])
			cur_wheel_pos = get_cur_wheel_pos(curW, cur_wheel_pos)
			print([Vx, Vy, W0], cur_robot_position, curW)
			
		publishJointData(joint_pub, cur_wheel_pos)
		rate.sleep()



if __name__ == "__main__":
	
	queue = Queue()
	queue.put([0, 0, 0, 0])
	encoder_process = Process(target=encoderFeedbackThread, name="omobotmovement", args=[queue])
	encoder_process.start()
	robotOdomPublisher(queue)