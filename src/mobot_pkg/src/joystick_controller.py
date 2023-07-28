#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rosparam, os, time, sys, signal
from std_msgs.msg import Int16MultiArray
from multiprocessing import Process, Queue

curLinSpeed = .80000000
curAngSpeed = .80000000
increments = .05000000
joy_msg = Twist()
refresh_rate = 20
max_speed_x = .14
max_speed_y = .14
max_speed_w = .7
minLinVel = .02
minANgVel = .01
max_wheel_rpm = [36, 36, 36, 36]
max_wheel_rpm_pos = [35.6, 35.9, 37.25, 36.15]
max_wheel_rpm_neg = [-35.3, -36.7, -36.18, -36.65]
max_wheel_speed_pos = [vals*2*3.1416/60 for vals in max_wheel_rpm_pos]
max_wheel_speed_neg = [vals*2*3.1416/60 for vals in max_wheel_rpm_neg]


ap = [-156.24596, -165.97551, -155.69695, -139.97149]
bp = [4.23815, 4.36329, 4.59189, 4.13725]
an = [-157.00181, -174.04037, -141.91634, -150.46193]
bn = [-4.27643, -4.4339, -4.2611, -4.49149]

L  = .1185
W  = .0825
SL = (L+W)
LS = 1/SL
r  = .0398
R  = 2*3.14159*r

try:
	os.system('echo y | rosnode cleanup')
	time.sleep(.1)
except:
	pass

def CheckSpeed(Lin_UP, Lin_DN, Rot_UP, Rot_DN):
	global curLinSpeed, curAngSpeed
	if Lin_UP and curLinSpeed < 1:
		curLinSpeed += increments
		if curLinSpeed > 1.0:
			curLinSpeed = 1.0
	elif Lin_DN and curLinSpeed >= .3:
		curLinSpeed -= increments
		if curLinSpeed < 0.3:
			curLinSpeed = 0.3

	if Rot_UP and curAngSpeed < .99:
		curAngSpeed += increments*.5
	elif Rot_DN and curAngSpeed >= .15:
		curAngSpeed -= increments*.5



def clearOffset(vx, vy, wo):
	if abs(vx) > -minLinVel and abs(vx) <minLinVel:
		vx = 0.0
	if abs(vy) > -minLinVel and abs(vy) <minLinVel:
		vy = 0.0
	if abs(wo) > -minANgVel and abs(wo) <minANgVel:
		wo = 0.0
	
	return [vx, vy, wo]

def joysticCB(joys, joy_queue):
	global curLinSpeed, curAngSpeed, joy_msg, cmd_vels, updated
	RS_X = joys.axes[3]
	RS_Y = joys.axes[4]
	LS_Y = joys.axes[0]
	LS_X = joys.axes[1]
	Lin_UP = joys.buttons[5]  
	Lin_DN = joys.buttons[3] 
	Rot_UP = joys.buttons[1]  
	Rot_DN = joys.buttons[2]
	CheckSpeed(Lin_UP, Lin_DN, Rot_UP, Rot_DN)
	if joys.buttons[6]:
		curLinSpeed = .80000000
		curAngSpeed = .80000000

	if joys.buttons[8]:
		curX = 0.0
		curY = 0.0
		curT = 0.0
	else:
		[curX, curY, curT] = clearOffset(LS_X*max_speed_x*curLinSpeed, LS_Y*max_speed_y*curLinSpeed,RS_X*max_speed_w*curAngSpeed)


	# print(curX, curY, curT)
	joy_queue.put([curX, curY, curT, curLinSpeed, curAngSpeed])
	cmd_vels = [curX, curY, curT]
	updated = True
	# print([curX, curY, curT], [curLinSpeed, curAngSpeed])
		

def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	os.system('echo y | rosnode cleanup')
	sys.exit(0)

def joystickProcess(joy_queue):
	global joy_pub
	rospy.init_node('joy_node', anonymous=True)
	rospy.Subscriber('/joy', Joy, joysticCB, callback_args=joy_queue, queue_size=5)
	signal.signal(signal.SIGINT, signal_handler)
	rospy.spin()


def getPwmFromAngSpeed(x, mode = 0):
	yp = [0, 0, 0, 0]
	yn = [0, 0, 0, 0]
	

	W = [0, 0, 0, 0]
	# x = [vals+.00000000000]
	if mode == 1:
		yp[0] = ap[0]/x[0] + (bp[0])
		yn[0] = an[0]/x[0] + (bn[0])
		yp[1] = ap[1]/x[1] + (bp[1])
		yn[1] = an[1]/x[1] + (bn[1])
		yp[2] = ap[2]/x[2] + (bp[2])
		yn[2] = an[2]/x[2] + (bn[2])
		yp[3] = ap[3]/x[3] + (bp[3])
		yn[3] = an[3]/x[3] + (bn[3])
	else:
		yp[0] = ap[0] / (x[0] - bp[0])
		yn[0] = an[0] / (x[0] - bn[0])
		yp[1] = ap[1] / (x[1] - bp[1])
		yn[1] = an[1] / (x[1] - bn[1])
		yp[2] = ap[2] / (x[2] - bp[2])
		yn[2] = an[2] / (x[2] - bn[2])
		yp[3] = ap[3] / (x[3] - bp[3])
		yn[3] = an[3] / (x[3] - bn[3])

	W = [yp[ind] if vals > 0 else yn[ind] for ind, vals in enumerate(x)]

	W = [int(vals) if abs(vals) > 45 else 0 for vals in  W]
	W_ = []
	for vals in W:
		if vals > 255: vals = 255
		elif vals < -255: vals = -255
		W_.append(vals)
	return W_

def Vxy2Angular(Vx, Vy, W0):
	W1 = (Vx - Vy - SL*W0) / r
	W2 = (Vx + Vy + SL*W0) / r
	W3 = (Vx + Vy - SL*W0) / r
	W4 = (Vx - Vy + SL*W0) / r
	
	W_ = [max_wheel_speed_pos[ind] if vals > max_wheel_speed_pos[ind] else vals for ind, vals in enumerate([W1, W2, W3, W4])]
	W = [max_wheel_speed_neg[ind] if vals < max_wheel_speed_neg[ind] else vals for ind, vals in enumerate(W_)]
	
	return W
		

publishing_frequency = 200
def joystickPublisher(joy_queue):
	global joy_msg, cmd_vels, updated, curLinSpeed, curAngSpeed

	rospy.init_node('joystick_node', anonymous=True)
	joy_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	pwm_pub = rospy.Publisher("/wheel_pwm", Int16MultiArray, queue_size=10)
	rate = rospy.Rate(publishing_frequency)

	wheel_msg = Int16MultiArray(); wheel_msg.data = [0, 0, 0, 0]; 
	pwm_pub.publish(wheel_msg)
	[joy_msg.linear.x, joy_msg.linear.y, joy_msg.angular.z] = [0, 0, 0]; joy_pub.publish(joy_msg)
	while not rospy.is_shutdown():
		# print("joy_msg")
		try:
			[joy_msg.linear.x, joy_msg.linear.y, joy_msg.angular.z, curLinSpeed, curAngSpeed] = joy_queue.get(timeout=1/refresh_rate)
			# joy_queue.put([joy_msg.linear.x, joy_msg.linear.y, joy_msg.angular.z])
		except:
			pass
		
		if updated:
			W_ = Vxy2Angular(joy_msg.linear.x, joy_msg.linear.y, joy_msg.angular.z)
			W = getPwmFromAngSpeed(W_)
			wheel_msg.data = W
			joy_pub.publish(joy_msg)
			pwm_pub.publish(wheel_msg)
			# print(rospy.Time.now().to_sec(), W)
			print([joy_msg.linear.x, joy_msg.linear.y, joy_msg.angular.z], W_)

		if [joy_msg.linear.x, joy_msg.linear.y, joy_msg.angular.z] == [0.0, 0.0, 0.0]:
			# print("False : ", [joy_msg.linear.x, joy_msg.linear.y, joy_msg.angular.z], [curLinSpeed, curAngSpeed])
			joy_pub.publish(joy_msg)
			pwm_pub.publish(wheel_msg)
			# print(W)
			updated = False
		else:
			# print([joy_msg.linear.x, joy_msg.linear.y, joy_msg.angular.z], [curLinSpeed, curAngSpeed])
			updated = True
		# print(joy_msg)

		rate.sleep()

		
cmd_vels = [0.0, 0.0, 0.0]
updated = False

rosparam.set_param("joy_node/dev", "/dev/input/js0")
os.system('rosrun joy joy_node &')
time.sleep(.1)

joy_queue = Queue()
joy_queue.put([0.0, 0.0, 0.0])
p = Process(target=joystickProcess, name="joystick_process", args=[joy_queue])
p.start()

joystickPublisher(joy_queue)