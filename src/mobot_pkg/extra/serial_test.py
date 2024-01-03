import serial
import math
from datetime import datetime as dtime
import os, time
from random import randint as rand
import sys, signal
from os import system
# os.system('echo  ${PASS_} | sudo usermod -a -G dialout shihab')
try:
	os.system('echo  ${PASS_} | sudo -S chmod 777 /dev/ttyTHS1')
except:
	pass
try:
	os.system('echo  ${PASS_} | sudo -S chmod 777 /dev/arduinoMot')
except:
	pass


print("Initiating..........")

arduino_enc_port = serial.Serial(
	port="/dev/ttyTHS1",
	baudrate=57600,
	bytesize=serial.EIGHTBITS,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	timeout=0.03
)


try:
	arduino_mot_port = serial.Serial(
		port="/dev/arduinoMot",
		baudrate=230400,
		bytesize=serial.EIGHTBITS,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		timeout=0.1
	)
except:
	pass



def signal_handler(sig, frame):
	global arduino_mot_port
	print('You pressed Ctrl+C!')
	try:
		arduino_mot_port.write('0,0,0,0\n'.encode('utf-8'))
		arduino_mot_port.write('0,0,0,0\n'.encode('utf-8'))
		arduino_mot_port.write('0,0,0,0\n'.encode('utf-8'))
		arduino_mot_port.write('0,0,0,0\n'.encode('utf-8'))
		system('echo y | rosnode cleanup\n'.encode('utf-8'))
	except Exception as e:
		print(e)
		pass
	
	sys.exit(0)



def serial_read():
	# print("Reading Data.......")
	global arduino_enc_port, arduino_mot_port
	ser_data = arduino_enc_port.readline().decode("utf-8")
	# if 
	
	# print("data", data)
	return ser_data 

cnt = 0; sentCnt = 0
refreshDelay = .100
prevSignal = '0,0,0,0\n'
while True:
	signal.signal(signal.SIGINT, signal_handler)
	motorSignal = '60,60,60,60\n'
	
	# datas = serial_read()
	# print("motorSignal : {}".format(motorSignal))
	# try:
	# 	arduino_mot_port.open()
	# except:
	# 	print("Couldn't open Mot port")

	try:
		if cnt >= 20:
			# print(cnt, "\t", "datas : {}".format(datas))
			cnt = 0
			motorSignal = '0,0,0,0\n'
			
		try:
			# write()
			if prevSignal != motorSignal:
				
				arduino_mot_port.write(motorSignal.encode('utf-8'))	
				signal.signal(signal.SIGINT, signal_handler)
				print(sentCnt, "\t>>\t" ,time.time(), " : \t", motorSignal)
				sentCnt += 1
			prevSignal = motorSignal
			# arduino_mot_port.close()
			cnt += 1			
		except Exception as e:
			print("Couldn't Write : ", e)
	except Exception as e:
		print("Couldn't Write>>>> : ", e)
	
	for dels in range(int(10)):
		signal.signal(signal.SIGINT, signal_handler)
		time.sleep(refreshDelay/10)

	


# while True:
# 	# datas = serial_read()
# 	print("Writing Data.......")
# 	arduino_enc_port.writelines('150,150,150,150,'.encode('utf-8'))
# 	time.sleep(2)
# 	arduino_enc_port.writelines('0,0,0,0,'.encode('utf-8'))
# 	time.sleep(10)
# 	arduino_enc_port.write('-150,-150,-150,-150,\n'.encode('utf-8'))
# 	time.sleep(2)
# 	arduino_enc_port.write('0,0,0,0,\n'.encode('utf-8'))
# 	time.sleep(2)
# 	arduino_enc_port.write('-150,150,150,-150,\n'.encode('utf-8'))
# 	time.sleep(2)
# 	arduino_enc_port.write('0,0,0,0,\n'.encode('utf-8'))
# 	time.sleep(2)
# 	arduino_enc_port.write('150,-150,-150,150,\n'.encode('utf-8'))
# 	time.sleep(2)
# 	arduino_enc_port.write('0,0,0,0,\n'.encode('utf-8'))
# 	time.sleep(5)

	# print(datas)