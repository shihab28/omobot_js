#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion, Vector3
from nav_msgs.msg import Odometry

def robot_state_cb(robot_datas):
    print(robot_datas)
def robot_state_listener():
    rospy.init_node("/robot_state_publisher", anonymous=True)
    rospy.Subscriber("tf_static", TFMessage, robot_state_cb)
    # rospy.Subscriber("tf", TFMessage, robot_state_cb)


def joint_state_cb(joint_data):
    # print(datas.transforms[0].transform.translation.x)
    print(joint_data)
def joint_state_listener():
    rospy.init_node("/joint_state_publisher_gui", anonymous=True)
    rospy.Subscriber("joint_states", JointState, joint_state_cb)


def teleop_cb(teleop_datas):
    print(teleop_datas)
def teleop_listener():
    rospy.init_node("/turtlebot3_teleop", anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, teleop_cb)

def odometry_cb(odom_datas):
    print(odom_datas)
def odometry_listener():
    rospy.init_node("odometry_publisher", anonymous=True)
    rospy.Subscriber("odom", Odometry, odometry_cb)


def listener():
    # teleop_listener()
    odometry_listener()
    rospy.spin()

if __name__ == "__main__":
    listener()
    