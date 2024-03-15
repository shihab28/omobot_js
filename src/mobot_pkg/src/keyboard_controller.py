#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy, signal
from os import system
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q   w   e
   a   s   d
   z   x   c

For Holonomic mode (strafing):
---------------------------
   u    i    o
   j    k    l
   m    ,    .

   or 

   7    8    9
   4    5    6
   1    2    3

   or press Shift for capital

   Q   W   E
   A   S   D
   Z   X   C  

   

t : z-up
b : z-down

anything else : stop

[/] : increase/decrease max speeds by 10%
=/- : increase/decrease only linear speed by 10%
9/0 : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
	
        'w':( 1, 0, 0, 0),
        'a':( 0, 0, 0, 1),        
        's':(-1, 0, 0, 0),
        'd':( 0, 0, 0,-1),

        'x':(-1, 0, 0, 0),

        'q':( 1, 0, 0, 1),
        'e':( 1, 0, 0,-1),        
        'z':(-1, 0, 0,-1),
        'c':(-1, 0, 0, 1),
        

        'u':( 1, 1, 0, 0),
        'i':( 1, 0, 0, 0),
        'o':( 1,-1, 0, 0),
        'j':( 0, 1, 0, 0),
        'k':( 0, 0, 0, 0),
        'l':( 0,-1, 0, 0),
        'm':(-1, 1, 0, 0),
        '<':(-1, 0, 0, 0),
        '>':(-1,-1, 0, 0),

		'7':( 1, 1, 0, 0),
        '8':( 1, 0, 0, 0),
        '9':( 1,-1, 0, 0),
        '4':( 0, 1, 0, 0),
        '5':( 0, 0, 0, 0),
        '6':( 0,-1, 0, 0),
        '1':(-1, 1, 0, 0),
        '2':(-1, 0, 0, 0),
        '3':(-1,-1, 0, 0),

        'Q':( 1, 1, 0, 0),
        'W':( 1, 0, 0, 0),
        'E':( 1,-1, 0, 0),
        'A':( 0, 1, 0, 0),
        'S':( 0, 0, 0, 0),
        'D':( 0,-1, 0, 0),
        'Z':(-1, 1, 0, 0),
        'X':(-1, 0, 0, 0),
        'C':(-1,-1, 0, 0),
		
        
        't':( 0, 0, 1, 0),
        'b':( 0, 0,-1, 0),
    }

speedBindings={
        '}':(1.1, 1.1),
        '{':(0.9, 0.9),
        '+':(1.1, 1.0),
        '_':(0.9, 1.0),
        ')':(1.0, 1.1),
        '(':(1.0, 0.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel_key', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()
        stamped = False
        twist_frame = "odom"
        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

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

def startKeyboardNode():
    settings = saveTerminalSettings()

    rospy.init_node('keyboard_node')

    speed = rospy.get_param("~speed", 0.1)
    turn = rospy.get_param("~turn", .6)
    speed_limit = rospy.get_param("~speed_limit", 1.4)
    turn_limit = rospy.get_param("~turn_limit", 81)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            signal.signal(signal.SIGINT, signal_handler)
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

if __name__=="__main__":
    startKeyboardNode()
    



	

	