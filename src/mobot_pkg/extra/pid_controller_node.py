#!/usr/bin/env python

import rospy 
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rosparam, os, time, sys, signal
from std_msgs.msg import Int16MultiArray
from multiprocessing import Process, Queue
from std_msgs.msg import String
from simple_pid import PID

encoder_topic = "/encoder_feedback"
encoder_node = "pid_controller_node"
max_wheel_rpm_pos = [35.6, 35.9, 37.25, 36.75]
max_wheel_rpm_neg = [-35.3, -36.1, -35.78, -35.05]
max_wheel_speed_pos = [vals*2*3.1416/60 for vals in max_wheel_rpm_pos]
max_wheel_speed_neg = [vals*2*3.1416/60 for vals in max_wheel_rpm_neg]
ap = [-156.24596, -165.97551, -155.69695, -153.1074]
bp = [4.23815, 4.36329, 4.59189, 4.14697]
an = [-157.00181, -166.04037, -141.91634, -160.6376]
bn = [-4.27643, -4.42413, -4.2651, -4.47939]
def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	os.system('echo y | rosnode cleanup')
	sys.exit(0)

L  = .1185
W  = .0825
SL = (L+W)
LS = 1/SL
r  = .0398
R  = 2*3.14159*r


def angularToCmdVel(W):
	Vx = round((W[0] + W[1] + W[2] + W[3]) * r / 4, 4)
	Vy = round((-W[0] + W[1] + W[2] - W[3]) * r / 4, 4)
	W0 = round((-W[0]*LS + W[1]*LS - W[2]*LS + W[3]*LS) * r / 4, 4)

	return [Vx, Vy, W0]

def encoderFeedbackCB(datas, queue):
	ppsp= [ 5100,  5125,  5325,  5185]
	ppsn= [-5065, -5230, -5175, -5270]
	# print(datas.data)
	W_speed_ = [round(int(vals)*max_wheel_speed_pos[ind]/ppsp[ind], 5) if int(vals) > 0 else round(int(vals) * max_wheel_speed_neg[ind]/ppsn[ind], 5) for ind, vals in enumerate(datas.data.strip().split(","))]
	W_speed = [0.0 if abs(vals) < 0.0001 else vals for vals in W_speed_]
	cmd_vel_ = angularToCmdVel(W_speed)
	queue.put(cmd_vel_)

def encoderFeedbackThread(queue):
	rospy.init_node("pid_controller_node", anonymous=True)
	print("Setting Up encoderFeedbackThread Node.......")
	rospy.Subscriber(encoder_topic, String, encoderFeedbackCB, callback_args=queue, queue_size=20)
	signal.signal(signal.SIGINT, signal_handler)	
	rospy.spin()


def setFeedbackCB(datas, queue):
	# print(datas)
	cmd_vel_ = [datas.linear.x, datas.linear.y, datas.angular.z]
	# print(datas)
	queue.put(cmd_vel_)

def setFeedbackThread(queue):
	rospy.init_node("pid_controller_node", anonymous=True)
	print("Setting Up setFeedbackThread Node.......")
	rospy.Subscriber("/cmd_vel_set", Twist, setFeedbackCB, callback_args=queue, queue_size=20)
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
	  


publishing_frequency = 20
wait_time = 1 / publishing_frequency # .005
max_speed_x = .5
max_speed_y = .5
max_speed_w = .4
def pidController(queue_cur, queue_set):
	global set_pid_controller
	rospy.init_node("pid_controller_node", anonymous=True)
	pwm_pub = rospy.Publisher("/cmd_vel_out", Twist, queue_size=10)
	# wheel_msg = Int16MultiArray()
	rate = rospy.Rate(publishing_frequency)
	W_SET = [0, 0, 0]
	W_PWM = [0, 0, 0]
	cmd_vel_msg = Twist()
	set_pid_controller = False
	prev_cmd_vel = [0.0, 0.0, 0.0]
	output_ = [0.0, 0.0, 0.0]
	VX_Controller = PID(Kp=.1, Kd=.05, Ki=.00, setpoint=prev_cmd_vel[0], output_limits=(-max_speed_x, max_speed_x))
	VY_Controller = PID(Kp=.1, Kd=.05, Ki=.00, setpoint=prev_cmd_vel[1], output_limits=(-max_speed_y, max_speed_y))
	W0_Controller = PID(Kp=.1, Kd=.05, Ki=.00, setpoint=prev_cmd_vel[1], output_limits=(-max_speed_w, max_speed_w))
	while not rospy.is_shutdown():
		try:
			W_PWM = queue_cur.get(timeout=wait_time)
		except:
			pass
		try:
			W_SET = queue_set.get(timeout=wait_time)
			[cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z] = W_SET
			VX_Controller.setpoint = W_SET[0]
			VY_Controller.setpoint = W_SET[1]
			W0_Controller.setpoint = W_SET[2]
		except:
			pass
			
		if W_SET == [0, 0, 0] or not set_pid_controller:
			pwm_pub.publish(cmd_vel_msg)
			prev_cmd_vel = W_SET

		else:		
			Vx_ = VX_Controller(W_PWM[0]) #, dt=1/publishing_frequency) #
			Vy_ = VY_Controller(W_PWM[1]) #, dt=1/publishing_frequency) #
			W0_ = W0_Controller(W_PWM[2]) #, dt=1/publishing_frequency) #
			output_ = [Vx_, Vy_, W0_]
			output_ = [vals if abs(vals)>.001 else 0.0 for vals in output_]
			[cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z] = [output_[ind]+W_SET[ind] for ind in range(3)]
			# print(rospy.Time.now().to_sec(), W_SET, W_PWM, output_)
		
		
		if prev_cmd_vel != output_:
			# print(rospy.Time.now().to_sec(), W_SET, W_PWM, output_)
			pwm_pub.publish(cmd_vel_msg)
			prev_cmd_vel = output_

		signal.signal(signal.SIGINT, signal_handler)
		rate.sleep()

set_pid_controller = False
if __name__ == "__main__":
	
	
	queue_set = Queue()
	queue_set.put([0, 0, 0])
	queue_cur = Queue()
	queue_cur.put([0, 0, 0])
	# set_subscriber = rospy.Subscriber()
	feedback_process = Process(target=encoderFeedbackThread, name="feedback_process_", args=[queue_cur])
	feedback_process.start()
	set_process = Process(target=setFeedbackThread, name="set_process_", args=[queue_set])
	set_process.start()
	pidController(queue_cur, queue_set)