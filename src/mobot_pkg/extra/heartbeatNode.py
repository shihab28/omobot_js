#!/usr/bin/env python2

import subprocess
import time
import os
import sys
import signal
# import Jetson.GPIO as GPIO

# GPIO.setmode(GPIO.BOARD)
gpio_pin = 23
# GPIO.setup(gpio_pin, GPIO.OUT, initial=GPIO.LOW)

pinMapping = {
	12: "79",
	18: "15",
	19: "16",
	21: "17",
	23: "18",
}


def pinModeInit(pinNumber=gpio_pin, mode='out'):
	exportCmd = "echo  ${PASS_} | sudo -S echo {} > /sys/class/gpio/export".format(pinMapping[pinNumber])
	os.system(exportCmd)
	time.sleep(2)
	modeCmd = "echo  ${PASS_} | sudo -S echo {} > /sys/class/gpio/gpio{}/direction".format(mode, pinMapping[pinNumber])
	os.system(modeCmd)
	print(exportCmd, "\n", modeCmd)


def digitalWrite(pinNumber=gpio_pin, value=0):
	os.system("echo  ${PASS_} | sudo -S echo {} > /sys/class/gpio/gpio{}/value".format(value, pinMapping[pinNumber]))


def releasePin(pinNumber=gpio_pin):
	os.system(
		"echo  ${PASS_} | sudo -S echo {} > /sys/class/gpio/unexport".format(pinMapping[pinNumber]))


pinModeInit(gpio_pin, 'out')


def signal_handler(sig, frame):
	print('You pressed Ctrl+C!')
	try:
		releasePin()
		os.system('echo y | rosnode cleanup')
	except:
		pass

	sys.exit(0)


def is_rosnode_running(node_names):
	try:
		# Use the `rosnode` command to list all running nodes
		try:
			output = subprocess.check_output(['rosnode', 'list'])
			running_nodes = output.decode().split('\n')
		except Exception as e:
			print(e)
			running_nodes = [""]
			
		# Check if the specified node name is in the list of running nodes
		for node_name in node_names:
			if '/' + node_name in running_nodes:
				pass
			else:
				return False
		return True
	except subprocess.CalledProcessError:
		# If the `rosnode` command fails, the ROS Master may not be running
		return False


if __name__ == '__main__':
	node_name_to_check = ['motor_node', 'arduino_serial']
	# print("Mode : ", GPIO.getmode())
	while True:
		runningVal = 1
		stoppedVal = 1 - runningVal

		try:
			if is_rosnode_running(node_name_to_check):
				print("The ROS node '{}' is     running {}".format(node_name_to_check, runningVal))
				digitalWrite(gpio_pin, runningVal)
			else:
				print("The ROS node '{}' is not running {}".format(node_name_to_check, stoppedVal))
				digitalWrite(gpio_pin, stoppedVal)
		except:
			print("The ROS node '{}' is not running {}".format(node_name_to_check, stoppedVal))
			digitalWrite(gpio_pin, stoppedVal)

		time.sleep(1)
		signal.signal(signal.SIGINT, signal_handler)
