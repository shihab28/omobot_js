#!/usr/bin/env python

import rospy, time, os, sys, signal

from math import sin, cos, pi
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf import transformations, TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from multiprocessing import Process, Queue

cur_pos = [0, 0, 0, 0]


try:
	os.system('echo y | rosnode cleanup')
	time.sleep(.1)
except:
	pass

def teleop_cb(cmdvel, self):

	self.cmdvel = cmdvel
	# print(cmdvel)
	self.cmdvel2Vxyw()

class OmobotMovementClass():
	def __init__(self, simpleQ):
		self.L  = .1185
		self.W  = .0825
		self.SL = (self.L*self.L+self.W*self.W)**.5
		self.LS = 1/self.SL
		self.r  = .0398
		self.R  = 2*3.14159*self.r
		self.Vx = 0
		self.Vy = 0
		self.W0 = 0.0
		self.W1 = 0.0
		self.W2 = 0.0
		self.W3 = 0.0
		self.W4 = 0.0
		self.cur_pos = [0.0, 0.0, 0.0, 0.0]
		self.simpleQ = simpleQ                                 


	def cmdvel2Vxyw(self):
		self.Vx = self.cmdvel.linear.x
		self.Vy = self.cmdvel.linear.y
		self.W0 = self.cmdvel.angular.z
		self.simpleQ.put([self.Vx, self.Vy, self.W0])
	
	def Angular2Vxyw(self):
		self.Vx = (self.W1 + self.W2 + self.W3 + self.W4) * self.R / 4
		self.Vy = (self.W1 - self.W2 - self.W3 + self.W4) * self.R / 4
		self.W0 = (-self.W1*self.LS + self.W2*self.LS - self.W3*self.LS + self.W4*self.LS) * self.R / 4

def Vxy2Angular(omobot, Vx, Vy, W0):
	W1 = (Vx + Vy - omobot.SL*W0) / omobot.R
	W2 = (Vx - Vy + omobot.SL*W0) / omobot.R
	W3 = (Vx - Vy - omobot.SL*W0) / omobot.R
	W4 = (Vx + Vy + omobot.SL*W0) / omobot.R
	# print([Vx, Vy, W0], end="\t")
	return [W1, W2, W3, W4]


def get_cur_wheel_pos(cur_pos, W1, W2, W3, W4):
	global prevTimeWhl
	curTime = rospy.Time.now().to_sec()
	passedTime = (curTime-prevTimeWhl)
	prevTimeWhl = curTime
	# print([W1, W2, W3, W4], end="\t")
	return [cur_pos[0]+W1*passedTime, cur_pos[1]+W2*passedTime, cur_pos[2]+W3*passedTime, cur_pos[3]+W4*passedTime]




def publishOdomData(odom_pub):
	global Vx, Vy, W0, cur_robot_position
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


def get_cur_robot_pos(cur_pos, Vx, Vy, W0):
	global prevTimePos

	curTime = rospy.Time.now().to_sec()
	dT = (curTime-prevTimePos)
	prevTimePos = curTime

	dW = W0 * dT
	W0 = cur_pos[2] + dW

	dx = (Vx * cos(W0) - Vy * sin(W0)) * dT
	dy = (Vx * sin(W0) + Vy * cos(W0)) * dT
	
	if dx!= 0.0 or dy!=0.0 or dW!=0.0:
		# print( [dx, dy, dW], end = "\t")
		pass
	return [cur_pos[0]+dx, cur_pos[1]+dy, W0]

def startTeleopThread(omobot):
	rospy.init_node("joystick_node", anonymous=True)
	# rospy.init_node("teleop_twist_keyboard", anonymous=True)
	rospy.Subscriber("/cmd_vel", Twist, teleop_cb, omobot, queue_size=20)
	signal.signal(signal.SIGINT, signal_handler)	
	rospy.spin()

def ombot_subscriber_publisher(omobot, queue):
	global prevTimeWhl, prevTimePos, cur_robot_position, Vx, Vy, W0
	rospy.init_node('joint_state_publisher', anonymous=True)
	pub = rospy.Publisher('/joint_states', JointState, queue_size=20)

	# rospy.init_node('odometry_publisher')
	odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

	rate = rospy.Rate(1000)
	rate.sleep()
	rospy.loginfo("Starting Publisher Node")
	prevTimePos = rospy.Time.now().to_sec()
	prevTimeWhl = rospy.Time.now().to_sec()
	prev_pos = [0, 0, 0, 0]
	cur_wheel_pos = [0, 0, 0, 0]
	cur_robot_position = [0.0, 0.0, 0.0]
	print("Starting Publisher Node")
	while not rospy.is_shutdown():
		prev_wheel_velocity = [0, 0, 0, 0]
		try:
			[Vx, Vy, W0] = queue.get(timeout=.002)
		
		# print([Vx, Vy, W0])
			cur_robot_position = get_cur_robot_pos(cur_robot_position, Vx, Vy, W0)
			if Vx!= 0.0 or Vy!=0.0 or W0!=0.0:
				print('pos : ', [Vx, Vy, W0], cur_robot_position)
			[W1, W2, W3, W4] = Vxy2Angular(omobot, Vx, Vy, W0)
			cur_wheel_pos = get_cur_wheel_pos(cur_wheel_pos, W1, W2, W3, W4)
			
			
		except:
			pass
		rate.sleep()
		

	# while not rospy.is_shutdown():


if __name__ == "__main__":
	
	queue = Queue()
	queue.put([0, 0, 0.0])
	omobot = OmobotMovementClass(queue)
	p = Process(target=startTeleopThread, name="omobotmovement", args=[omobot])
	p.start()
	ombot_subscriber_publisher(omobot, queue)








'''
rosnode info /robot_state_publisher 
--------------------------------------------------------------------------------
Node [/robot_state_publisher]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Subscriptions: 
 * /joint_states [sensor_msgs/JointState]

Services: 
 * /robot_state_publisher/get_loggers
 * /robot_state_publisher/set_logger_level
'''

'''
name: [wheel_lf_joint, wheel_rf_joint, wheel_lb_joint, wheel_rb_joint]
position: [0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []
header: 
  seq: 4937
  stamp: 
	secs: 1687367081
	nsecs: 505572080
  frame_id: ''
'''






'''
Node [/joint_state_publisher]
Publications: 
 * /joint_states [sensor_msgs/JointState]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: None

Services: 
 * /joint_state_publisher/get_loggers
 * /joint_state_publisher/set_logger_level


contacting node http://shihab:45405/ ...
Pid: 31203
Connections:
 * topic: /joint_states
	* to: /robot_state_publisher
	* direction: outbound (43053 - 127.0.0.1:60746) [10]
	* transport: TCPROS
 * topic: /rosout
	* to: /rosout
	* direction: outbound (43053 - 127.0.0.1:60742) [8]
	* transport: TCPROS
'''




'''
Node [/joint_state_publisher_gui]
Publications: 
 * /joint_states [sensor_msgs/JointState]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: None

Services: 
 * /joint_state_publisher_gui/get_loggers
 * /joint_state_publisher_gui/set_logger_level


contacting node http://shihab:43255/ ...
Pid: 32020
Connections:
 * topic: /joint_states
	* to: /robot_state_publisher
	* direction: outbound (41571 - 127.0.0.1:42074) [19]
	* transport: TCPROS
 * topic: /rosout
	* to: /rosout
	* direction: outbound (41571 - 127.0.0.1:42058) [8]
	* transport: TCPROS
'''




'''
Node [/turtlebot3_teleop]
Publications: 
 * /cmd_vel [geometry_msgs/Twist]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: None

Services: 
 * /turtlebot3_teleop/get_loggers
 * /turtlebot3_teleop/set_logger_level


contacting node http://shihab:38717/ ...
Pid: 28990
Connections:
 * topic: /rosout
	* to: /rosout
	* direction: outbound (38949 - 127.0.0.1:34020) [8]
	* transport: TCPROS
'''