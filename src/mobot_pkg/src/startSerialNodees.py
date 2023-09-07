#!/usr/bin/env python


import rospy, signal
from rosserial_arduino import SerialClient
from serial import SerialException
from time import sleep
from os import system
import serial
import time
import sys
from multiprocessing import Process, Queue
from std_msgs.msg import String
from std_msgs.msg import Float32


try:
	system("echo y | rosnode cleanup\n")
	time.sleep(1)
except:
	try:
		system("echo y | rosnode cleanup\n")
		time.sleep(1)
	except:
		pass




def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	try:
		system('echo y | rosnode cleanup\n')
	except:
		pass
	
	sys.exit(0)

def startMotorNode():
	rospy.init_node("motor_node")
	rospy.loginfo("ROS Serial Python Node")

	port_mot = rospy.get_param('~port','/dev/arduinoMot')
	# port_enc = rospy.get_param('~port','/dev/arduinoEnc')
	baud = int(rospy.get_param('~baud','115200'))

	# Number of seconds of sync failure after which Arduino is auto-reset.
	# 0 = no timeout, auto-reset disabled
	auto_reset_timeout = int(rospy.get_param('~auto_reset_timeout','0'))

	# for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
	# TIOCM_DTR_str) line, which causes an IOError, when using simulated port
	fix_pyserial_for_test = rospy.get_param('~fix_pyserial_for_test', False)

	# TODO: do we really want command line params in addition to parameter server params?
	sys.argv = rospy.myargv(argv=sys.argv)
	if len(sys.argv) >= 2 :
		port_mot  = sys.argv[1]

	while not rospy.is_shutdown():
		rospy.loginfo("Connecting to %s at %d baud" % (port_mot, baud))

		signal.signal(signal.SIGINT, signal_handler)
		try:
			client_mot = SerialClient(port_mot, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
			client_mot.run()
			# client_mot = SerialClient(port_enc, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
			# client_mot.run()
		# except KeyboardInterrupt:
		# 	break
		except SerialException:
			sleep(1.0)
			continue
		except OSError:
			sleep(1.0)
			continue



refresh_rate = 40
vlotageRatio = 12.16/748
voltageOn = True
def startEncoderSerialNode():
	try:
		system("echo 1628 | sudo -S chmod 777 /dev/ttyTHS1")
		# system("echo 1628 | sudo -S chmod 777 /dev/arduinoEnc")
		# port_enc = rospy.get_param('~port','/dev/arduinoEnc')
	except Exception as e:
		print(e)


	# arduino_enc_port = serial.Serial(port='/dev/arduinoEnc', baudrate=115200, timeout=.05)
	arduino_enc_port = serial.Serial(port='/dev/ttyTHS1', baudrate=57600, timeout=.05)
	def serial_read():
		data = arduino_enc_port.read_until('\n')
		# print("data", data)
		return str(data).strip().strip().split(",")
	
	

	rospy.init_node("arduino_serial", anonymous=False)
	enc_publisher = rospy.Publisher('/encoder_feedback', String, queue_size=5)
	voltage_publisher = rospy.Publisher('/battery_voltage', Float32, queue_size=5)
	rate = rospy.Rate(refresh_rate)
	strData = String()
	voltageData = Float32()
	strData.data = '0,0,0,0'
	print("Started Encoder Feedback Serial ttyTHS1.....")
	while not rospy.is_shutdown():
		signal.signal(signal.SIGINT, signal_handler)
		value = serial_read()
		# print(value, value)
		if len(value) == 5 and voltageOn:
			strData.data = ','.join(value[:4])
			voltageData.data = float(value[4])*vlotageRatio
			enc_publisher.publish(strData) # printing the value
			voltage_publisher.publish(voltageData) # printing the value
		else:
			strData.data = ','.join(value[:4])
			enc_publisher.publish(strData)
			voltageData.data = 0.00
			voltage_publisher.publish(voltageData)
		
		print(rospy.Time().now().to_sec(), strData.data)#,  voltageData.data)

		rate.sleep()

def main():
	process_mot = Process(target=startMotorNode, name="joystick_process")
	process_mot.start()
	process_enc = Process(target=startEncoderSerialNode, name="encoder_process")
	process_enc.start()

if __name__=="__main__":
	main()









































# def startEncoderNode():
# 	rospy.init_node("encoder_node")
# 	rospy.loginfo("ROS Serial Python Node")

# 	# port_enc = rospy.get_param('~port','/dev/arduinoEnc')
# 	port_enc = rospy.get_param('~port','/dev/arduinoEnc')
# 	baud = int(rospy.get_param('~baud','115200'))
# 	# try:
# 	#     system("echo 1628 | sudo -S chmod 777 /dev/ttyTHS1")
# 	# except Exception as e:
# 	#     print(e)
		
# 	# port_enc = rospy.get_param('~port','/dev/ttyTHS1')
# 	# baud = int(rospy.get_param('~baud','57600'))

# 	# Number of seconds of sync failure after which Arduino is auto-reset.
# 	# 0 = no timeout, auto-reset disabled
# 	auto_reset_timeout = int(rospy.get_param('~auto_reset_timeout','0'))

# 	# for systems where pyserial yields errors in the fcntl.ioctl(self.fd, TIOCMBIS, \
# 	# TIOCM_DTR_str) line, which causes an IOError, when using simulated port
# 	fix_pyserial_for_test = rospy.get_param('~fix_pyserial_for_test', False)

# 	# TODO: do we really want command line params in addition to parameter server params?
# 	sys.argv = rospy.myargv(argv=sys.argv)
# 	if len(sys.argv) >= 2 :
# 		port_enc  = sys.argv[1]

# 	while not rospy.is_shutdown():
# 		rospy.loginfo("Connecting to %s at %d baud" % (port_enc, baud))
# 		try:
# 			# client_mot = SerialClient(port_mot, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
# 			# client_mot.run()
# 			client_mot = SerialClient(port_enc, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
# 			client_mot.run()
# 		# except KeyboardInterrupt:
# 		# 	break
# 		except SerialException:
# 			sleep(1.0)
# 			continue
# 		except OSError:
# 			sleep(1.0)
# 			continue


	