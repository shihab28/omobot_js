#!/usr/bin/env python2

'''
Owner Information:
Author: Shihab Uddin Ahamad
Email: shihab.ahamad28@gmail.com
Date: 03/20/2024

Description:
This script integrates joystick and keyboard inputs to control the Omobot. It listens to joystick inputs, converts them to Twist messages (linear and angular velocities, `cmd_vel_joy`, `cmd_vel_key`) , and publishes the Twist mesdsage for the robot's movement. Concurrently, it manages PWM signals node for the motor control. The script also manages the launching of necessary ROS nodes and handling system signals for graceful shutdowns.
'''

# Importing necessary libraries and modules
import rospy , roslaunch
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rosparam, os, time, sys, signal
from multiprocessing import Process
import wheel_pwm_controller
import keyboard_controller

# Global variables for speed and message publishing
curLinSpeed = .80000000
curAngSpeed = .80000000
increments = .05000000
joy_msg = Twist()
refresh_rate = 20
max_speed_x = .14
max_speed_y = .14
max_speed_w = .81
minLinVel = .02
minANgVel = .01
publishing_frequency = 500
updated = False

# RPM and speed settings for the wheels
max_wheel_rpm = [36, 36, 36, 36]
max_wheel_rpm_pos = [35.3, 37.7, 36.4, 34.9]# [ 35.2,  34.90,  36.1,  35.2]
max_wheel_rpm_neg = [-35.1, -35.3, -34.7, -33.1]# [-34.9, -36.15, -35.5, -36.1]
max_wheel_speed_pos = [vals*2*3.1416/60 for vals in max_wheel_rpm_pos]
max_wheel_speed_neg = [vals*2*3.1416/60 for vals in max_wheel_rpm_neg]

# Launching necessary ROS nodes and launch file for multipliplexing inputs for joystick control and Twist message publication
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
twstMuxCli = ['./src/mobot_pkg/launch/twist_mux.launch','vel:=2.19']
twistMux_launch_args = twstMuxCli[1:]
twistMux_launch_file =  [(roslaunch.rlutil.resolve_launch_arguments(twstMuxCli)[0], twistMux_launch_args)]
parent = roslaunch.parent.ROSLaunchParent(uuid, twistMux_launch_file)
parent.start()


# Cleaning up ROS nodes upon start
try:
	os.system('echo y | rosnode cleanup')
	time.sleep(.1)
except:
	pass

# Signal handling for graceful shutdown
def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	os.system('echo y | rosnode cleanup')
	sys.exit(0)


# When the velocities (Vx, Vy, W0) are smaller than the minimum velocities, assign them to zero
def clearOffset(vx, vy, wo):
	if abs(vx) > -minLinVel and abs(vx) <minLinVel:
		vx = 0.0
	if abs(vy) > -minLinVel and abs(vy) <minLinVel:
		vy = 0.0
	if abs(wo) > -minANgVel and abs(wo) <minANgVel:
		wo = 0.0
	return [vx, vy, wo]


# Class to handle joystick inputs and publish corresponding Twist messages
class ControllerClass():
    
    # Initialization of ROS node, publishers, and subscribers
	def __init__(self):
		self.node = rospy.init_node('joystick_node', anonymous=False)
		self.joy_pub = rospy.Publisher('/cmd_vel_joy', Twist, queue_size=1)
		self.joy_sub = rospy.Subscriber('/joy', Joy, self.joysticCB, queue_size=10)
		self.rate = rospy.Rate(publishing_frequency)

		self.joy_msg = Twist()
		self.cmd_vel = [0.0, 0.0, 0.0]
		self.setCmdVel()
		self.publishCmdVel()
		self.updated = updated
		self.refresh_rate = 20
		self.curLinSpeed = .80000000
		self.curAngSpeed = .80000000
		self.increments = .05000000
	
 	# Publish the computed Twist message based on joystick inputs
	def publishCmdVel(self):
		self.joy_pub.publish(self.joy_msg)

	# Set the Twist message values based on joystick input processing
	def setCmdVel(self):
		[self.joy_msg.linear.x, self.joy_msg.linear.y, self.joy_msg.angular.z] = self.cmd_vel

	# Regulates the speed sensitivity of the joystick input
	def regulateSpeed(self):
		if self.Lin_UP and self.curLinSpeed < 1:
			self.curLinSpeed += self.increments
			if self.curLinSpeed > 1.0:
				self.curLinSpeed = 1.0
		elif self.Lin_DN and self.curLinSpeed >= .3:
			self.curLinSpeed -= self.increments
			if self.curLinSpeed < 0.3:
				self.curLinSpeed = 0.3

		if self.Rot_UP and self.curAngSpeed < .99:
			self.curAngSpeed += self.increments*.5
		elif self.Rot_DN and self.curAngSpeed >= .15:
			self.curAngSpeed -= self.increments*.5

 	# Callback for joystick input. Processes the input, convert, and updates command velocities
	def joysticCB(self, joys):
		RS_X = joys.axes[2]  	 		#joys.axes[3]
		RS_Y = joys.axes[5]  	 		#joys.axes[4]
		LS_Y = joys.axes[0]  	 		#joys.axes[0]
		LS_X = joys.axes[1]  	 		#joys.axes[1]
		ANG_XY = joys.buttons[6] 		#joys.axes[2]
		self.Lin_UP = joys.buttons[3] 	#joys.buttons[5]  
		self.Lin_DN = joys.buttons[1] 	#joys.buttons[3] 
		self.Rot_UP = joys.buttons[2] 	#joys.buttons[1]  
		self.Rot_DN = joys.buttons[0] 	#joys.buttons[2]

		mov_front_back = joys.axes[7]
		mov_left_right = joys.axes[6]
		mov_back_only = joys.buttons[8]
		self.regulateSpeed()
		
		if joys.buttons[9]:
			self.curLinSpeed = .80000000
			self.curAngSpeed = .80000000

		if ANG_XY:
			if   LS_X > .01 and LS_Y > .01:  [curX, curY, curT] = clearOffset(max_speed_x*curLinSpeed, max_speed_y*curLinSpeed, 0.0)
			elif LS_X < -.01 and LS_Y > .01:  [curX, curY, curT] = clearOffset(-max_speed_x*curLinSpeed, max_speed_y*curLinSpeed, 0.0)
			elif LS_X > .01 and LS_Y < -.01:  [curX, curY, curT] = clearOffset(max_speed_x*curLinSpeed, -max_speed_y*curLinSpeed, 0.0)
			elif LS_X < -.01 and LS_Y < -.01:  [curX, curY, curT] = clearOffset(-max_speed_x*curLinSpeed, -max_speed_y*curLinSpeed, 0.0)
			else: [curX, curY, curT] = [0, 0, 0.0]

		elif mov_back_only > .8:
			[curX, curY, curT] = clearOffset(-mov_back_only*max_speed_x*curLinSpeed, 0, 0)
		elif abs(mov_front_back) > .8:
			[curX, curY, curT] = clearOffset(mov_front_back*max_speed_x*curLinSpeed, 0, 0)
		elif abs(mov_left_right) > .8:
			[curX, curY, curT] = clearOffset(0, mov_left_right*max_speed_x*curLinSpeed,0)
		else:
			[curX, curY, curT] = clearOffset(LS_X*max_speed_x*curLinSpeed, LS_Y*max_speed_y*curLinSpeed,RS_X*max_speed_w*curAngSpeed)

		self.cmd_vel = [curX, curY, curT]
		self.updated = True



# Handles joystick input, converting it to Twist messages for Omobot control
def joystickPublisher():
	joystick_controller = ControllerClass()

	print("joystickPublisher node init.............")
	while not rospy.is_shutdown():
		try:
			joystick_controller.setCmdVel()
		except:
			pass

		if joystick_controller.updated:
			joystick_controller.publishCmdVel()
			print("joy_msg : ", joystick_controller.joy_msg.linear)

		if joystick_controller.cmd_vel == [0.0, 0.0, 0.0]:
			joystick_controller.updated = False
		else:
			joystick_controller.updated = True
		
		signal.signal(signal.SIGINT, signal_handler)
		joystick_controller.rate.sleep()


# Calls the `wheel_pwm_controller.py` script to manage PWM signals for motor control.
def pwmProcess():
	wheel_pwm_controller.startPwmProcess()


# Main function to initialize the control system
def startControllerScript():

 	# Sets the device parameters for the joystick
	rosparam.set_param("joy_node/dev", "/dev/input/js0")
	os.system('rosrun joy joy_node &')
	time.sleep(.1)

	# Starts the joystick controller subprocess
	j = Process(target=joystickPublisher, name="joystickProcess", args=[])
	j.start() 
	
 	# Starts the pwm controller subprocess
	w = Process(target=pwmProcess, name="pwmProcess", args=[])
	w.start()

 	# Starts the keyboard controller subprocess
	keyboard_controller.startKeyboardNode()	
 
 
if __name__ == "__main__":
	startControllerScript()