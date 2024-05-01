#!/usr/bin/env python2

import rospy, time, os, sys, signal
from geometry_msgs.msg import Twist
import math
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    os.system('echo y | rosnode cleanup')
    sys.exit(0)



global_frame = "map"
base_frame = 'base_footprint'
class PathPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pathPublisherNode')
        self.robot_path_pub = rospy.Publisher('/omobot_path', Path, queue_size=5)
        self.robot_path = Path()
        self.robot_path.header.frame_id =  global_frame
        self.tf_listener = tf.TransformListener()
        
        self.num_points = 360
        self.center_x = 0.0
        self.center_y = 0.0
        self.radius = .7
        self.Vx = .07
        
        self.circular_path_pub = rospy.Publisher('/circle_trajectory', Path, queue_size=5)
        self.circular_path = Path()
        self.circular_path.header.frame_id = global_frame  
        self.angle_step = 2 * math.pi / self.num_points
        self.generateCircularPath()
        self.circular_path_pub.publish(self.circular_path)
        self.rate = rospy.Rate(20)
        
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = self.Vx
        self.cmd_vel.angular.z = self.Vx/self.radius
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

    def start(self):
    
        while not rospy.is_shutdown():
            try:
                # Check if the transformation is available at the current time
                if self.tf_listener.canTransform(global_frame, base_frame, rospy.Time(0)):
                    (trans, rot) = self.tf_listener.lookupTransform(global_frame, base_frame, rospy.Time(0))
                    print(trans, rot)
                    
                    pose = PoseStamped()
                    pose.header.frame_id = self.robot_path.header.frame_id
                    pose.header.stamp = self.robot_path.header.stamp
                    pose.pose.position.x = trans[0]
                    pose.pose.position.y = trans[1]
                    # pose.pose.orientation.x = rot[0] 
                    # pose.pose.orientation.y = rot[1] 
                    # pose.pose.orientation.z = rot[2] 
                    pose.pose.orientation.w = 1.0 
                    self.robot_path.poses.append(pose)
                    if len(self.robot_path.poses) >= self.num_points*10:
                        self.robot_path.poses.pop(0)
                    self.robot_path_pub.publish(self.robot_path)
                    self.circular_path_pub.publish(self.circular_path)
                    self.publishCmdVel()
        
                else:
                    print("Transformation from base_footprint to map not available.")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF error: {}".format(e))

            signal.signal(signal.SIGINT, signal_handler)
            self.rate.sleep()
    
    
        
    def generateCircularPath(self):
        for i in range(self.num_points):
            angle = self.angle_step * i
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)

            pose = PoseStamped()
            pose.header.frame_id = self.circular_path.header.frame_id
            pose.header.stamp = self.circular_path.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0  # assuming no orientation change
            self.circular_path.poses.append(pose)
            
    def publishCmdVel(self):
        pass
        self.cmd_vel_pub.publish(self.cmd_vel)
        
                
        

if __name__ == '__main__':
    path_publisher = PathPublisher()
    path_publisher.start()

