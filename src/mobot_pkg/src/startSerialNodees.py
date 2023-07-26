#!/usr/bin/env python


import rospy
from rosserial_arduino import SerialClient
from serial import SerialException
from time import sleep

import sys
from multiprocessing import Process, Queue

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
        try:
            client_mot = SerialClient(port_mot, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
            client_mot.run()
            # client_mot = SerialClient(port_enc, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
            # client_mot.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue

def startEncoderNode():
    rospy.init_node("encoder_node")
    rospy.loginfo("ROS Serial Python Node")

    # port_mot = rospy.get_param('~port','/dev/arduinoMot')
    port_enc = rospy.get_param('~port','/dev/arduinoEnc')
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
        port_enc  = sys.argv[1]

    while not rospy.is_shutdown():
        rospy.loginfo("Connecting to %s at %d baud" % (port_enc, baud))
        try:
            # client_mot = SerialClient(port_mot, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
            # client_mot.run()
            client_mot = SerialClient(port_enc, baud, fix_pyserial_for_test=fix_pyserial_for_test, auto_reset_timeout=auto_reset_timeout)
            client_mot.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue

def main():
    process_mot = Process(target=startMotorNode, name="joystick_process")
    process_mot.start()
    process_enc = Process(target=startEncoderNode, name="joystick_process")
    process_enc.start()
    process_mot.join()
    process_enc.join()

if __name__=="__main__":
    main();
    