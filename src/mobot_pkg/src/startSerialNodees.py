#!/usr/bin/env python2


'''
Owner Information:
Author: Shihab Uddin Ahamad
Email: shihab.ahamad28@gmail.com
Date: 03/20/2024

Description:
This script serves as a communication layer between a Jetson device and an Arduino, facilitating real-time control and feedback for the Omobot robot's motors. It listens for encoder feedback through serial communication, processes this data to understand the robot's movement, and publishes it to the ROS ecosystem. Simultaneously, it subscribes to velocity commands (cmd_vel_str) and forwards them to the Arduino, enabling precise motor control based on ROS messages.

Key Features:
- Real-time serial communication with Arduino for motor control and feedback.
- Subscription to ROS topics for velocity commands and publication of encoder feedback.
- Dynamic permission management for serial ports to ensure smooth communication.
- Integration of signal handling for graceful shutdown and process management.

Workflow:
1. Perform initial setup by cleaning up existing ROS nodes and setting permissions for serial ports.
2. Establish serial communication with the Arduino, dedicated to motor control and encoder feedback.
3. Initialize ROS node and subscribers to listen for cmd_vel_str messages containing velocity commands.
4. In a separate process, continuously read encoder feedback from the Arduino and publish it to a ROS topic.
5. Process and send received cmd_vel_str messages to the Arduino for motor control.
6. Implement signal handling to ensure resources are properly released upon script termination.
'''

# Import necessary libraries and modules
import rospy, signal, time, sys, serial
from os import system
from multiprocessing import Process
from std_msgs.msg import String, Float32
import getEncoderData


# Initialization of constants and variables.
refresh_rate = 40
vlotageRatio = 12.22/740
voltageOn = True
currentRatio = 3.323/1023#0.62/506
currentVoltageRatio = .1
currentOn = True


# System setup for serial port permissions
try:
	system("echo y | rosnode cleanup\n")
	time.sleep(.5)
except:
	
	pass
try:
	system("echo $PASS_ | sudo -S chmod 777 /dev/arduinoMot\n")
	time.sleep(.5)
except:
	try:
		system("echo $PASS_ | sudo -S chmod 777 /dev/ttyUSB0\n")
		time.sleep(.2)
		system("echo $PASS_ | sudo -S chmod 777 /dev/ttyUSB1\n")
		time.sleep(.3)
	except:
		
		pass
try:
	system("echo $PASS_ | sudo -S chmod 777 /dev/ttyUSB0\n")
	time.sleep(.2)
	system("echo $PASS_ | sudo -S chmod 777 /dev/ttyUSB1\n")
	time.sleep(.3)
except:
		try:
			system("echo $PASS_ | sudo -S chmod 777 /dev/ttyUSB0\n")
			time.sleep(.2)
			system("echo $PASS_ | sudo -S chmod 777 /dev/ttyUSB1\n")
			time.sleep(.3)
		except:

			pass


# Serial port initialization for Arduino communication
try:
	arduino_mot_port = serial.Serial(
		port="/dev/arduinoMot",
		baudrate=230400,
		bytesize=serial.EIGHTBITS,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		timeout=0.05
	)
except:
	pass


# Signal handler for graceful shutdown
def signal_handler(sig, frame):
	global arduino_mot_port
	print('You pressed Ctrl+C!')
	try:
		arduino_mot_port.write('0,0,0,0\n')
		arduino_mot_port.write('0,0,0,0\n')
		arduino_mot_port.write('0,0,0,0\n')
		arduino_mot_port.write('0,0,0,0\n')
		system('echo y | rosnode cleanup\n')
	except Exception as e:
		print(e)
		pass
	
	sys.exit(0)


# Callback for processing and sending velocity commands to Arduino
def motorFeedbackCB(wheelData, arduino_mot_port):
	datas = wheelData.data
	print("Sending Data to Arduino : ", datas)
	arduino_mot_port.write(datas.encode('utf-8'))


# Main function for motor control node
def startMotorNode():
	global arduino_mot_port

	motor_node = "motor_node"
	try:
		arduino_mot_port = serial.Serial(
			port="/dev/arduinoMot",
			baudrate=230400,
			bytesize=serial.EIGHTBITS,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			timeout=0.05
		)
	except:
		try:
			arduino_mot_port = serial.Serial(
				port="/dev/arduinoMot",
				baudrate=230400,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				timeout=0.05
			)
		except:
			pass
	rospy.init_node(motor_node, anonymous=False)

	# global prevTimeWhl, prevTimePos

	##########
	print("Initiating Motor Node................")
	rospy.Subscriber("/wheel_pwm_str", String, motorFeedbackCB, callback_args=arduino_mot_port, queue_size=1)
	signal.signal(signal.SIGINT, signal_handler)	
	signal.signal(signal.SIGINT, signal_handler)
	rospy.spin()
	






# Function for handling encoder feedback and publishing it
def startEncoderSerialNode():
	try:
		system("echo  ${PASS_} | sudo -S chmod 777 /dev/ttyTHS1")
	except Exception as e:
		print(e)

	arduino_enc_port = serial.Serial(
	port="/dev/ttyTHS1",
	baudrate=57600,
	bytesize=serial.EIGHTBITS,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	timeout=.05
)
	
	def serial_read():
		data = arduino_enc_port.read_until('\n')
		return str(data).strip().strip().split(",")

	rospy.init_node("arduino_serial", anonymous=False)
	enc_publisher = rospy.Publisher('/encoder_feedback', String, queue_size=1)
	voltage_publisher = rospy.Publisher('/battery_voltage', Float32, queue_size=1)
	current_publisher = rospy.Publisher('/battery_current', Float32, queue_size=1)
	rate = rospy.Rate(refresh_rate)

	strData = String()
	strData.data = '0,0,0,0'
	voltageData = Float32()
	currentData = Float32()
	print("Started Encoder Feedback Serial ttyTHS1.....")

	enc_publisher.publish(strData)

	while not rospy.is_shutdown():
		signal.signal(signal.SIGINT, signal_handler)
		value = []
		serialData = serial_read()

		value = serialData
		try:
			W_speed = [int(vals) for vals in value]
		
		# print(value)
			if len(value) == 5 and voltageOn:
				strData.data = ','.join(value[:4])
				try:
					voltageData.data = round(float(value[4])*vlotageRatio, 3)
				except Exception as e:
					print(e, value)
					voltageData.data = 0.0
				currentData.data = 0.00

			elif len(value) == 6 and voltageOn and currentOn:
				strData.data = ','.join(value[:4])

				try:
					voltageData.data = round(float(value[4])*vlotageRatio, 3)
				except Exception as e:
					print(e, value)
					voltageData.data = 0.0
				try:
					currentData.data = round( (float(value[5])*currentRatio - 1.661) / currentVoltageRatio, 3)
				except Exception as e:
					print(e, value)
					currentData.data = 0.0

			else:
				strData.data = ','.join(value[:4])
				voltageData.data = 0.00
				currentData.data = 0.00
			
			enc_publisher.publish(strData)
			voltage_publisher.publish(voltageData)
			current_publisher.publish(currentData)
		except:
			pass
		
		print("Serial Node : ", strData.data, voltageData.data, currentData.data)
		
		rate.sleep()

# Main function for initialization and process management for encoder feedback and motor control
def main():
	process_mot = Process(target=startMotorNode, name="motor_process")
	process_mot.start()
	process_enc = Process(target=getEncoderData.startGetEncoderNode, name="encoder_process")
	process_enc.start()
	startEncoderSerialNode()

if __name__=="__main__":
	main()

