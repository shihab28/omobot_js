#!/usr/bin/env python

import rospy, os
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, LinkStates
from gazebo_msgs.srv import SetModelState, SetLinkState
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from tf import transformations
from multiprocessing import Process, Queue
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import std_msgs, String
import serial
os.system('echo  ${PASS_} | sudo -S chmod 777 /dev/arduinoMot')
arduinoMot = serial.Serial(
    port="/dev/arduinoMot",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

L  = .1185
W  = .0825
SL = (L+W)
LS = 1/SL
r  = .0398
R  = 2*3.14159*r

# os.system("rosrun rosserial_python serial_node.py /dev/arduinoMot")
# os.system("rosrun rosserial_python serial_node.py /dev/ArduinoEnc")


def Vxy2Angular(Vx, Vy, W0):
	W1 = (Vx + Vy - SL*W0) / r
	W2 = (Vx - Vy + SL*W0) / r
	W3 = (Vx - Vy - SL*W0) / r
	W4 = (Vx + Vy + SL*W0) / r

	
	[W1, W2, W3, W4] = [0.0 if abs(vals) < .001 else vals for vals in [W1, W2, W3, W4]]

	return [W1, W2, W3, W4]

def Sxy2Angular(Sx, Sy, Rw):
	
	S1 = (Sx + Sy - SL*Rw) / r
	S2 = (Sx - Sy + SL*Rw) / r
	S3 = (Sx - Sy - SL*Rw) / r
	S4 = (Sx + Sy + SL*Rw) / r
	return [S1, S2, S3, S4]

def velocity_callback(msg):
	# Extract the linear and angular velocities from the message
	Vx = msg.linear.x
	Vy = msg.linear.y
	W0 = msg.angular.z
	# Calculate the individual wheel velocities for an omni-directional robot
	linkState = LinkStates()
	
	velocity_msg = ModelState()
	velocity_msg.model_name = 'omobot'
	[W1, W2, W3, W4] = Vxy2Angular(Vx, Vy, W0)
	# velocity_msg.twist.angular

	# Set the wheel velocities using the gazebo_msgs/SetModelState service
	# set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', velocity_msg)    

prev_Sx = 0.0
prev_Sy = 0.0
prev_Rw = 0.0

W1 = 0.0
W2 = 0.0
W3 = 0.0
W4 = 0.0

S = [0.0, 0.0, 0.0, 0.0]

def tf_callback(msg):
	global prev_tm, prev_Sx, prev_Sy, prev_Rw, W1, W2, W3, W4, S
	for transform in msg.transforms:
		# print(transform)
		if transform.child_frame_id == 'base_footprint':  # Replace 'base_link' with the appropriate frame ID for your robot
			cur_tm = rospy.Time.now().to_sec()
			dt = cur_tm - prev_tm
			prev_tm = cur_tm
			if dt > .0001:
				Sx = transform.transform.translation.x
				Sy = transform.transform.translation.y
				R_Quat = transform.transform.rotation
				Rx, Ry, Rw = transformations.euler_from_quaternion([R_Quat.x, R_Quat.y, R_Quat.z, R_Quat.w])
				if Rw < 0: Rw -= 6.283185
				Vx = (Sx - prev_Sx)/dt
				Vy = (Sy - prev_Sy)/dt
				W0 = (Rw - prev_Rw)/dt
				[Vx, Vy, W0] = [0.0 if abs(vals) < .001 else vals for vals in [Vx, Vy, W0]]
				
				W = [0.0 if abs(vals) < .001 else vals for vals in Vxy2Angular(Vx, Vy, W0)]
				S = [W[ind]*dt+S[ind] for ind in range(len(W))]

				pub_arduino = rospy.Publisher('/arduino_wheel_velocity', String, queue_size=20)
				velocity_msg = str(W[0])+'_'+str(W[1])+'_'+str(W[2])+'_'+str(W[3])+'\n'
				pub_arduino.publish(velocity_msg)


				pub_joint_state = rospy.Publisher('/joint_states', JointState, queue_size=20)
				joint_msg = JointState()
				joint_msg.header = Header()
				joint_msg.header.stamp = rospy.Time.now()
				joint_msg.name = ["wheel_lf_joint", "wheel_rf_joint", "wheel_lb_joint", "wheel_rb_joint"]
				joint_msg.position = S
				joint_msg.velocity = []
				joint_msg.effort = []

				print(S)
				pub_joint_state.publish(joint_msg)

				prev_Sx = Sx
				prev_Sy = Sy
				prev_Rw = Rw

def odom_callback(odom):
	global prev_tm, S
	cur_tm = rospy.Time.now().to_sec()
	dt = cur_tm - prev_tm
	prev_tm = cur_tm
	# print("odom : ", odom.twist)
	Vx = odom.twist.twist.linear.x
	Vy = odom.twist.twist.linear.y
	W0 = odom.twist.twist.angular.z

	[Vx, Vy, W0] = [0.0 if abs(vals) < .0001 else vals for vals in [Vx, Vy, W0]]
	W = [0.0 if abs(vals) < .001 else vals for vals in Vxy2Angular(Vx, Vy, W0)]
	S = [W[ind]*dt+S[ind] for ind in range(len(W))]

	pub_arduino = rospy.Publisher('/arduino_wheel_velocity', String, queue_size=20)
	velocity_msg = str(W[0])+'_'+str(W[1])+'_'+str(W[2])+'_'+str(W[3])+'\n'
	pub_arduino.publish(velocity_msg)
	try:
		arduinoMot.write(bytes(velocity_msg, 'utf-8'))
	except:
		print("Couldn't send data to arduino : ", dt)

	pub_joint_state = rospy.Publisher('/joint_states', JointState, queue_size=20)
	joint_msg = JointState()
	joint_msg.header = Header()
	joint_msg.header.stamp = rospy.Time.now()
	joint_msg.name = ["wheel_lf_joint", "wheel_rf_joint", "wheel_lb_joint", "wheel_rb_joint"]
	joint_msg.position = S
	joint_msg.velocity = []
	joint_msg.effort = []

	print(S)
	pub_joint_state.publish(joint_msg)

queue = Queue()
queue.put([0, 0, 0.0])
refresh_rate = 20

def TfListenerNode(queue):
	global prev_tm
	rospy.init_node('tf_listener')

	prev_tm = rospy.Time.now().to_sec()
	# rospy.Subscriber('/odom', TFMessage, tf_callback, callback_args=queue)
	rospy.Subscriber('/odom', Odometry, odom_callback)
	rospy.spin()

def JointStatePublisherNode(queue):
	rospy.init_node('joint_state_publisher', anonymous=True)
	rate = rospy.Rate(refresh_rate)
	[S1, S2, S3, S4] = [0.0, 0.0, 0.0, 0.0]

	while not rospy.is_shutdown():
		S = queue.get(timeout=1/refresh_rate)



def main():
	global prev_tm
	rospy.init_node('tf_listener')

	prev_tm = rospy.Time.now().to_sec()
	# rospy.Subscriber('/odom', TFMessage, tf_callback, callback_args=queue)
	rospy.Subscriber('/odom', Odometry, odom_callback)
	rospy.spin()
	
	



if __name__ == '__main__':
	main()



