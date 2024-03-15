#!/usr/bin/env python2


import rospy, signal
from rosserial_arduino import SerialClient
import serial
from serial import SerialException
from time import sleep
from os import system
import time
import sys
from multiprocessing import Process, Queue
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int16MultiArray
import getEncoderData

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





def motorFeedbackCB(wheelData, arduino_mot_port):
	datas = wheelData.data
	print("Sending Data to Arduino : ", datas)
	arduino_mot_port.write(datas.encode('utf-8'))


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
	



refresh_rate = 40
vlotageRatio = 12.22/740
voltageOn = True
currentRatio = 3.323/1023#0.62/506
currentVoltageRatio = .1
currentOn = True


def startEncoderSerialNode():
	try:
		system("echo  ${PASS_} | sudo -S chmod 777 /dev/ttyTHS1")
		# system("echo  ${PASS_} | sudo -S chmod 777 /dev/arduinoEnc")
		# port_enc = rospy.get_param('~port','/dev/arduinoEnc')
	except Exception as e:
		print(e)


	# arduino_enc_port = serial.Serial(port='/dev/arduinoEnc', baudrate=115200, timeout=.05)
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
	samples = 4
	sampleCnt = 0
	totalVals = [0, 0, 0, 0, 0, 0]
	prevVals = [0, 0, 0, 0, 0, 0]
	testOn = False

	enc_publisher.publish(strData)

	# encoder_odom_process = Process(target=getEncoderData.startGetEncoderScript, name="encoder_odom_process")
	# encoder_odom_process.start()

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
		
		# print("Serial Node : ", rospy.Time().now().to_sec(), strData.data, voltageData.data, voltageData.data)#,  voltageData.data)
		print("Serial Node : ", strData.data, voltageData.data, currentData.data)#,  voltageData.data)
		


		
		# signal.signal(signal.SIGINT, signal_handler)
		rate.sleep()
	# signal.signal(signal.SIGINT, signal_handler)

def main():
	process_mot = Process(target=startMotorNode, name="motor_process")
	process_mot.start()
	process_enc = Process(target=getEncoderData.startGetEncoderNode, name="encoder_process")
	process_enc.start()
	startEncoderSerialNode()

if __name__=="__main__":
	main()

