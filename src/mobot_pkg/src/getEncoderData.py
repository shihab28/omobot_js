#!/usr/bin/env python2

'''
Owner Information:
Author: Shihab Uddin Ahamad
Email: shihab.ahamad28@gmail.com
Date: 03/20/2024

Description:
This script facilitates feedback control for the Omobot. It computes real-time wheel positions and velocities from encoder feedback, translates these into odometry information, and publishes the updated robot state for navigation and control. This enables the Omobot to accurately track its movement over time, recalculate its path, and interact autonomously.

Key Features:
- Real-time processing of encoder feedback for all wheels.
- Calculation of wheel positions and speeds.
- Translation of wheel speeds into robot's linear and angular velocities.
- Generation of odometry messages reflecting the robot's current state.
- Broadcasting of the robot's transform from odometry to the robot base frame `base_footprint`.
- Publishing of joint state messages for further integration with ROS control and visualization tools like RViz.

Workflow:
1. Initialize the ROS node and subscribers to listen for encoder feedback.
2. On receiving encoder data, calculate the individual wheel speeds and overall robot velocities.
3. Continuously update the robot's current position and orientation based on computed data.
4. Publish odometry messages to the '/odom' topic, providing a comprehensive view of the robot's movement.
5. Publish joint state messages to the '/joint_states' topic for compatibility with ROS visualization and control frameworks.
6. Handles system signals for graceful shutdown and resource cleanup.
'''

# Import necessary Python and ROS libraries
import rospy, os, sys, signal
from tf import transformations, TransformBroadcaster
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from math import sin, cos
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


# Robot physical and kinematic parameters
L  = .1185          # Distance between left and right wheels' origin
W  = .0825          # Distance between front and back wheels' origin
SL = (L+W)
LS = 1/SL
r  = .0398          # Wheel radius
R  = 2*3.14159*r    # Wheel circumference


# Maximum wheel speeds in RPM for forward and reverse directions
max_wheel_rpm_pos =  [ 35.2,  34.90,  36.8,  34.5] 
max_wheel_rpm_neg =  [-34.9, -36.15, -35.5, -36.1] 

# Conversion of RPM to rad/s
max_wheel_speed_pos = [vals*2*3.1416/60 for vals in max_wheel_rpm_pos]
max_wheel_speed_neg = [vals*2*3.1416/60 for vals in max_wheel_rpm_neg]

# Pulses per second for positive and negative directions
ppsp = [ 5120, 5120, 5120, 5120]  
ppsn = [-5046,-5335,-5036,-5206]


# Refresh rate for processing encoder feedback and publishing state information
refresh_rate = 40


# Signal handler for graceful shutdown
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    os.system('echo y | rosnode cleanup')
    sys.exit(0)


# Function to convert angular wheel velocities to robot's cmd_vel
def angularToCmdVel(W):
    # Convert wheel speeds (W) to robot's linear (Vx, Vy) and angular (W0) velocities
    Vx = round((W[0] + W[1] + W[2] + W[3]) * r / 4, 4)
    Vy = round((-W[0] + W[1] + W[2] - W[3]) * r / 4, 4)
    W0 = round((-W[0]*LS + W[1]*LS - W[2]*LS + W[3]*LS) * r / 4, 4)
    return Vx, Vy, W0


# Main class for odometry publishing
class OdomentryPublisher():
    
    # ROS node initialization, publishers, subscribers, and state variables
    def __init__(self):
        self.node = rospy.init_node("encoder_node", anonymous=False)
        self.curW = [0, 0, 0, 0]
        self.W_speed = [0, 0, 0, 0]
        self.Vx, self.Vy, self.W0 = 0.0, 0.0, 0.0
        self.cmd_vel = self.Vx, self.Vy, self.W0
        self.cur_robot_position = [0, 0, 0]
        self.cur_wheel_pos = [0, 0, 0, 0]
        self.odom_pub		= rospy.Publisher('/odom', Odometry, queue_size=10)
        self.vel_feed_pub 	= rospy.Publisher('/velocity_feedback', String, queue_size=10)
        self.joint_pub 		= rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.enc_sub        =rospy.Subscriber("/encoder_feedback", String, self.encoderFeedbackCB, queue_size=10)
        self.rate = rospy.Rate(refresh_rate)
        self.prevTimeWhl = rospy.Time.now().to_sec()
        self.prevTimePos = rospy.Time.now().to_sec()
        self.rate.sleep()
    
    # Callback for processing encoder feedback and updating robot velocities and real-time position.
    def encoderFeedbackCB(self, datas):
        try:
            W_speed_ = [round(int(vals)*max_wheel_speed_pos[ind]/ppsp[ind], 8) if int(vals) > 0 else round(int(vals) * max_wheel_speed_neg[ind]/ppsn[ind], 8) for ind, vals in enumerate(datas.data.strip().split(","))]
            self.W_speed = [0.0 if abs(vals) < 0.0001 else vals for vals in W_speed_]
        except:
            print("Couldn't get Feedback, making feedback array [0, 0, 0, 0]")
            self.W_speed = [0, 0, 0, 0]
        self.Vx, self.Vy, self.W0 = angularToCmdVel(self.W_speed)
        self.cmd_vel = self.Vx, self.Vy, self.W0
        self.getCurRobotPos()
        
    # Calculate the robot's current position based on velocities
    def getCurRobotPos(self):
        Vx, Vy, W0 = self.Vx, self.Vy, self.W0
        cur_pos = self.cur_robot_position
        curTime = rospy.Time.now().to_sec()
        dT = (curTime-self.prevTimePos)
        W_ang =  cur_pos[2]
        dW = W0 * dT
        dx = (Vx * cos(W_ang) - Vy * sin(W_ang)) * dT
        dy = (Vx * sin(W_ang) + Vy * cos(W_ang)) * dT
        
        self.prevTimePos = curTime
        self.cur_robot_position = [cur_pos[0]+dx, cur_pos[1]+dy, cur_pos[2] + dW]
    
    
    # Update wheel positions based on current wheel speeds
    def getCurWheelPos(self):
        W = self.W_speed
        if W != None and  W != [0.0, 0.0, 0.0, 0.0] and len(W) == 4:
            curTime = rospy.Time.now().to_sec()
            passedTime = (curTime-self.prevTimeWhl)
            self.prevTimeWhl = curTime
            self.cur_wheel_pos =  [self.cur_wheel_pos[0]+W[0]*passedTime, self.cur_wheel_pos[1]+W[1]*passedTime, self.cur_wheel_pos[2]+W[2]*passedTime, self.cur_wheel_pos[3]+W[3]*passedTime]
    
    # Publish odometry data reflecting the robot's current state
    def publishOdomData(self):
        Vx, Vy, W0 = self.Vx, self.Vy, self.W0
        odom_quat = transformations.quaternion_from_euler(0, 0, self.cur_robot_position[2])
        odom_broadcaster = TransformBroadcaster()
        cur_time = rospy.Time.now()
        odom_broadcaster.sendTransform(
            (self.cur_robot_position[0], self.cur_robot_position[1], 0.),
            odom_quat,
            cur_time,
            "base_footprint",
            "odom"
        )
        odom = Odometry()
        odom.header.stamp = cur_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(Vx, Vy, 0), Vector3(0, 0, W0))
        self.odom_pub.publish(odom)
        self.vel_feed_pub.publish(str("{},{},{}".format(Vx, Vy, W0)))
    
    # Publish joint state data for visualization and further control
    def publishJointData(self):
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ["wheel_lf_joint", "wheel_rf_joint", "wheel_lb_joint", "wheel_rb_joint"]
        joint_msg.position = self.cur_wheel_pos
        joint_msg.velocity = []
        joint_msg.effort = []
        self.joint_pub.publish(joint_msg)


# Main function to start the odometry publisher node
def startGetEncoderNode():
    odometryPublisher = OdomentryPublisher()
    odometryPublisher.rate.sleep()

    while not rospy.is_shutdown():
        
        odometryPublisher.getCurWheelPos()
        odometryPublisher.publishOdomData()
        odometryPublisher.publishJointData()
        
        signal.signal(signal.SIGINT, signal_handler)
        odometryPublisher.rate.sleep()

if __name__ == "__main__":
    startGetEncoderNode()