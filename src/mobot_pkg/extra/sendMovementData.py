import serial
import math
from datetime import datetime as dtime
import os, time
from random import randint as rand
# os.system('echo  ${PASS_} | sudo usermod -a -G dialout shihab')
os.system('echo  ${PASS_} | sudo -S chmod 777 /dev/arduinoMot')
# os.system('"1628" | sudo chmod 777 /dev/ttyTHS1')
print("Initiating..........")

arduinoMot = serial.Serial(
    port="/dev/arduinoMot",
    baudrate=230800,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)



angs_ = {0: 0, 1:225, 2:270, 3:315, 4:180, 5:0, 6:0, 7:135, 8:90, 9:4}


DataLoggingDict = {}


print("Initiated")
testDistance = False
if testDistance:
    startTime = dtime.now().microsecond//1000
    print('Start Time : ', startTime)
    runTime = 5000

    # while (dtime.now().microsecond//1000 - startTime) < runTime:
    for i in range(1):
        valStr = f'1628,1628,1628\n'
        arduinoMot.write(bytes(valStr, 'utf-8'))
        time.sleep(15)
        valStr = f'0,0,0\n'
        arduinoMot.write(bytes(valStr, 'utf-8'))
        endTime = dtime.now().microsecond//1000
        print(endTime)
        time.sleep(10)    
    
else:
    while True:
        # for spd in range(100, 950, 10):
            
        # data = None
        sep = ","
        spd = 0
        i = 0
        # print(i)
        angs = 0
        ang = int(input("give direction : "))
        if ang > 0 and ang < 10 and ang != 5:
            i= angs_[ang]
            print(i)
            spd = 100

            vx = spd * math.sin(i*3.14/180)
            vy = spd * math.cos(i*3.14/180)
            w0 = 0
            valStr = f'{vx},{vy},{w0},\n'
        elif ang == 79:
            valStr = f'{0},{0},{.3},\n'
        elif ang == 97:
            valStr = f'{0},{0},{-.3},\n'
        else:
            valStr = f'{0},{0},{0},\n'

        
        

        for j in range(2):
            arduinoMot.write(bytes(valStr, 'utf-8'))
            time.sleep(.01)


        # arduinoMot.write(b'0,0\n')    