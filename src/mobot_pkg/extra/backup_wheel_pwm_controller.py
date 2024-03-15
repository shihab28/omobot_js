#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rosparam, os, time, sys, signal
from std_msgs.msg import Int16MultiArray, String
from multiprocessing import Process, Queue

curLinSpeed = .80000000
curAngSpeed = .80000000
increments = .05000000
pwm_msg = Twist()
refresh_rate = 10
max_speed_x = .14
max_speed_y = .14
max_speed_w = .7
minLinVel = .02
minANgVel = .01
max_wheel_rpm_pos = [ 35.2,  34.90,  36.1,  35.2]
max_wheel_rpm_neg = [-34.9, -36.15, -35.5, -36.1]
max_wheel_speed_pos = [vals*2*3.1416/60 for vals in max_wheel_rpm_pos]
max_wheel_speed_neg = [vals*2*3.1416/60 for vals in max_wheel_rpm_neg]
max_wheel_pulse_pos = [ 5045,  5023,  5193,  5055]
max_wheel_pulse_neg = [-5037, -5197, -5099, -5211]


L  = .1185
W  = .0825
SL = (L+W)
LS = 1/SL
r  = .0398
R  = 2*3.14159*r

# ap = [-156.24596, -165.97551, -155.69695, -139.97149]
# bp = [4.23815, 4.36329, 4.59189, 4.13725]
# an = [-157.00181, -174.04037, -141.91634, -150.46193]
# bn = [-4.27643, -4.4339, -4.2611, -4.49149]

# ap = [-150.61793, -158.4042, -165.29882, -153.1074]
# bp = [4.27373, 4.3164, 4.56555, 4.14697]
# an = [-148.47458, -166.4961, -151.14043, -160.6376]
# bn = [-4.24042, -4.42413, -4.24936, -4.47939]

# ap = [-156.24596, -165.97551, -150.69695, -145.1074]
# bp = [   4.23815,    4.31929,    4.59189,   4.44374]
# an = [-157.00181, -164.04037, -141.91634, -150.1376]
# bn = [  -4.5643,   -4.20051,   -4.16936,  -4.46939]	

# ap = [-143.21693, -146.35979, -130.29756, -136.70724]
# bp = [4.31598, 4.29486, 4.36087, 4.27206]
# an = [-139.48209, -154.50459, -123.10459, -134.60437]
# bn = [-4.22564, -4.41595, -4.24997, -4.34679]

ap = [-188.20627, -173.32657, -218.38976, -153.79884]
bp = [4.52182, 4.31568, 4.76275, 4.05594]
an = [-183.36784, -205.12899, -191.50974, -183.83753]
bn = [-4.37894, -4.67605, -4.26249, -4.4918]

cmd_vels = [0.0, 0.0, 0.0]
updated = False
max_pwm = 250


def getPwmFromAngSpeed(x, mode = 1):
	yp = [0, 0, 0, 0]
	yn = [0, 0, 0, 0]
	W = [0, 0, 0, 0]
	# x = [vals+.00000000000]
	if mode == 1:
		yp[0] = ap[0] / (x[0] - bp[0])
		yn[0] = an[0] / (x[0] - bn[0])

		yp[1] = ap[1] / (x[1] - bp[1])
		yn[1] = an[1] / (x[1] - bn[1])
		yp[2] = ap[2] / (x[2] - bp[2])
		yn[2] = an[2] / (x[2] - bn[2])

		yp[3] = ap[3] / (x[3] - bp[3])
		yn[3] = an[3] / (x[3] - bn[3])

	else:
		yp[0] = ap[0]/x[0] + (bp[0])
		yn[0] = an[0]/x[0] + (bn[0])
		yp[1] = ap[1]/x[1] + (bp[1])
		yn[1] = an[1]/x[1] + (bn[1])
		yp[2] = ap[2]/x[2] + (bp[2])
		yn[2] = an[2]/x[2] + (bn[2])
		yp[3] = ap[3]/x[3] + (bp[3])
		yn[3] = an[3]/x[3] + (bn[3])
		

	W = [yp[ind] if vals > 0 else yn[ind] for ind, vals in enumerate(x)]

	W = [int(vals) if abs(vals) > 70 else 0 for vals in  W]
	W_ = []
	for vals in W:
		if vals > max_pwm: vals = max_pwm
		elif vals < -max_pwm: vals = -max_pwm
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
		
try:
	os.system('echo y | rosnode cleanup')
	time.sleep(.1)
except:
	pass
def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	os.system('echo y | rosnode cleanup')
	sys.exit(0)

def clearOffset(vx, vy, wo):
	if abs(vx) > -minLinVel and abs(vx) <minLinVel:
		vx = 0.0
	if abs(vy) > -minLinVel and abs(vy) <minLinVel:
		vy = 0.0
	if abs(wo) > -minANgVel and abs(wo) <minANgVel:
		wo = 0.0
	
	return [vx, vy, wo]

publishing_frequency = 10
def pwmCB(cmdVelMsg):
	global pwm_pub, pwm_pub_str
	wheel_msg = Int16MultiArray(); wheel_msg.data = [0, 0, 0, 0]
	Vx, Vy, W0 = cmdVelMsg.linear.x, cmdVelMsg.linear.y, cmdVelMsg.angular.z
	W_ = Vxy2Angular(Vx, Vy, W0)
	W_PWM = getPwmFromAngSpeed(W_)
	wheel_msg.data = W_PWM
	if W_PWM != [0, 0, 0, 0]:
		print("Joystick Node : ", rospy.Time().now().to_sec(), W_PWM)
	
	pwm_pub.publish(wheel_msg)

	wheel_msg_str = String()
	temp_msg = ''
	for vals in W_PWM:
		temp_msg += str(vals) + ','
	temp_msg += '\n'
	wheel_msg_str.data = temp_msg
	pwm_pub_str.publish(wheel_msg_str)

def pwmCB_test(cmdVelMsg, pwm_pub):
	global max_pwm
	wheel_msg = Int16MultiArray(); wheel_msg.data = [0, 0, 0, 0]
	W_PWM = [0, 0, 0, 0]
	Vx, Vy, W0 = cmdVelMsg.linear.x, cmdVelMsg.linear.y, cmdVelMsg.angular.z
	max_pwm = int(max_pwm)
	if Vx > .1:
		W_PWM = [max_pwm, max_pwm, max_pwm, max_pwm]
	elif Vx < -.1:
		W_PWM = [-max_pwm, -max_pwm, -max_pwm, -max_pwm]
	
	print("Joystick Node : ", rospy.Time().now().to_sec(), W_PWM)
	wheel_msg.data = W_PWM
	pwm_pub.publish(wheel_msg)


def velocityCB(datas, queue):
	V_xyw = str(datas.data).strip().split(',')
	if len(V_xyw) == 3:
		V_xyw = [float(vels) for vels in V_xyw]
	# queue.put(V_xyw)
	print(V_xyw)



test_on = False
def pwmProcess(pwm_queue=None):
	global pwm_pub, wheel_msg, pwm_pub_str, wheel_msg_str
	queue = Queue()
	queue.put([0.0, 0.0, 0.0])
	rospy.init_node('pwm_node', anonymous=False)
	pwm_pub = rospy.Publisher("/wheel_pwm", Int16MultiArray, queue_size=2)
	pwm_pub_str = rospy.Publisher("/wheel_pwm_str", String, queue_size=2)
	wheel_msg = Int16MultiArray()
	wheel_msg_str = String()
	rate = rospy.Rate(publishing_frequency)

	rospy.Subscriber('/cmd_vel_out', Twist, pwmCB, queue_size=5)
	# rospy.Subscriber('/velocity_feedback', String, velocityCB, callback_args=queue, queue_size=2)

	# if not test_on:
	# else:
	# 	rospy.Subscriber('/cmd_vel_set', Twist, pwmCB_test, callback_args=pwm_pub, queue_size=5)
	
	signal.signal(signal.SIGINT, signal_handler)
	rospy.spin()



def startPwmProcess():
	pwmProcess()

if __name__ == "__main__":
	pwmProcess()