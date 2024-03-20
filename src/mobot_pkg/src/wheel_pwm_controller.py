#!/usr/bin/env python


# Import necessary libraries.
import rospy 
from geometry_msgs.msg import Twist
import os, time, sys, signal
from std_msgs.msg import Int16MultiArray, String


# Initialization of constants and variables for speed control and PWM calculation.
curLinSpeed = .80000000
curAngSpeed = .80000000
increments = .05000000
pwm_msg = Twist()
refresh_rate = 10
max_speed_x = .14
max_speed_y = .14
max_speed_w = .7
minLinVel = .02
minANgVel = .01
max_wheel_rpm_pos = [ 35.2,  34.90,  36.1,  35.2]
max_wheel_rpm_neg = [-34.9, -36.15, -35.5, -36.1]

# Convert RPM to rad/s for positive and negative directions.
max_wheel_speed_pos = [vals*2*3.1416/60 for vals in max_wheel_rpm_pos]
max_wheel_speed_neg = [vals*2*3.1416/60 for vals in max_wheel_rpm_neg]
max_wheel_pulse_pos = [ 5045,  5023,  5193,  5055]
max_wheel_pulse_neg = [-5037, -5197, -5099, -5211]

# Robot physical parameters for kinematic calculations.
L  = .1185
W  = .0825
SL = (L+W)
LS = 1/SL
r  = .0398
R  = 2*3.14159*r

# Parameters for PWM calculation identified for the motor system model. 
ap = [-188.20627, -173.32657, -218.38976, -153.79884]
bp = [4.52182, 4.31568, 4.76275, 4.05594]
an = [-183.36784, -205.12899, -191.50974, -183.83753]
bn = [-4.37894, -4.67605, -4.26249, -4.4918]


cmd_vels = [0.0, 0.0, 0.0]
updated = False
max_pwm = 250
publishing_frequency = 10
test_on = False


# Cleanup and signal handling for graceful shutdown.
try:
    os.system('echo y | rosnode cleanup')
    time.sleep(.1)
except:
    pass
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    os.system('echo y | rosnode cleanup')
    sys.exit(0)


# Function to calculate PWM values for each wheel from angular speeds.
def getPwmFromAngSpeed(x, mode = 1):
    yp = [0, 0, 0, 0]
    yn = [0, 0, 0, 0]
    W = [0, 0, 0, 0]
    if mode == 1:
        yp[0] = ap[0] / (x[0] - bp[0])
        yn[0] = an[0] / (x[0] - bn[0])

        yp[1] = ap[1] / (x[1] - bp[1])
        yn[1] = an[1] / (x[1] - bn[1])
        yp[2] = ap[2] / (x[2] - bp[2])
        yn[2] = an[2] / (x[2] - bn[2])

        yp[3] = ap[3] / (x[3] - bp[3])
        yn[3] = an[3] / (x[3] - bn[3])

    else:
        yp[0] = ap[0]/x[0] + (bp[0])
        yn[0] = an[0]/x[0] + (bn[0])
        yp[1] = ap[1]/x[1] + (bp[1])
        yn[1] = an[1]/x[1] + (bn[1])
        yp[2] = ap[2]/x[2] + (bp[2])
        yn[2] = an[2]/x[2] + (bn[2])
        yp[3] = ap[3]/x[3] + (bp[3])
        yn[3] = an[3]/x[3] + (bn[3])
        

    W = [yp[ind] if vals > 0 else yn[ind] for ind, vals in enumerate(x)]

    W = [int(vals) if abs(vals) > 70 else 0 for vals in  W]
    W_ = []
    
    # Restrict the speed between max and minimum allowable pwm value.
    for vals in W:
        if vals > max_pwm: vals = max_pwm
        elif vals < -max_pwm: vals = -max_pwm
        W_.append(vals)
    return W_


# Converts the linear and angular velocity command velocity [Vx, Vy, W0] into wheel's anhular speeds W in rads^-1.
def Vxy2Angular(Vx, Vy, W0):
    W1 = (Vx - Vy - SL*W0) / r
    W2 = (Vx + Vy + SL*W0) / r
    W3 = (Vx + Vy - SL*W0) / r
    W4 = (Vx - Vy + SL*W0) / r
    
    W_ = [max_wheel_speed_pos[ind] if vals > max_wheel_speed_pos[ind] else vals for ind, vals in enumerate([W1, W2, W3, W4])]
    W = [max_wheel_speed_neg[ind] if vals < max_wheel_speed_neg[ind] else vals for ind, vals in enumerate(W_)]
    return W


# Funtion to clear minor offsets in velocity commands to avoid jitter or unwanted movements.
def clearOffset(vx, vy, wo):
    
    if abs(vx) > -minLinVel and abs(vx) <minLinVel:
        vx = 0.0
    if abs(vy) > -minLinVel and abs(vy) <minLinVel:
        vy = 0.0
    if abs(wo) > -minANgVel and abs(wo) <minANgVel:
        wo = 0.0
    
    return [vx, vy, wo]


# WheelController class defines a ROS node for controlling the robot's wheels.
class WheelController():
    # Initialize the ROS node, publishers, and subscriber.
    def __init__(self):
        self.node = rospy.init_node('pwm_node', anonymous=False)
        self.pwm_pub = rospy.Publisher("/wheel_pwm", Int16MultiArray, queue_size=2)
        self.pwm_pub_str = rospy.Publisher("/wheel_pwm_str", String, queue_size=2)
        self.wheel_msg = Int16MultiArray()
        self.wheel_msg_str = String()
        self.rate = rospy.Rate(publishing_frequency)
        self.vel_sub = rospy.Subscriber('/cmd_vel_out', Twist, self.pwmCB, queue_size=5)
    
    
    # Callback function for the cmd_vel subscriber. Calculates and publishes PWM values.
    def pwmCB(self, cmdVelMsg):
        self.wheel_msg.data = [0, 0, 0, 0]
        Vx, Vy, W0 = cmdVelMsg.linear.x, cmdVelMsg.linear.y, cmdVelMsg.angular.z
        W_ = Vxy2Angular(Vx, Vy, W0)    # Calculates the wheel's rotational velocity
        W_PWM = getPwmFromAngSpeed(W_)  # Calculates PWM values for corrosponding wheel's rotational velocity
        self.wheel_msg.data = W_PWM
        if W_PWM != [0, 0, 0, 0]:
            print("Joystick Node : ", rospy.Time().now().to_sec(), W_PWM)
        self.pwm_pub.publish(self.wheel_msg)    # Publishes the PWM values
    
    
        # Convert the PWM values as string seperated by "," and publish as String topic
        temp_msg = ''
        for vals in W_PWM:
            temp_msg += str(vals) + ','
        temp_msg += '\n'
        self.wheel_msg_str.data = temp_msg
        self.pwm_pub_str.publish(self.wheel_msg_str)
    
    
    # Starts the ROS node and handles signal interrupt for shutdown.
    def run(self):
        signal.signal(signal.SIGINT, signal_handler)
        rospy.spin()


# Initializes and runs the WheelController.
def startPwmProcess():
    wheelController = WheelController()
    wheelController.run()

if __name__ == "__main__":
    startPwmProcess()
