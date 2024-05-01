#!/usr/bin/env python2

import rospy, time, os, sys, signal
from geometry_msgs.msg import Twist
import math
import tf
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def signal_handler(sig, frame):
    global path_publisher
    print('You pressed Ctrl+C!')
    os.system('echo y | rosnode cleanup')
    try:
        path_publisher.cmd_vel_pub.publish(Twist())
        path_publisher.cmd_vel_pub.publish(Twist())
    except:
        pass
    sys.exit(0)



global_frame = "odom"
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
        self.radius = .5
        self.mode = "noController"
        
        if self.mode == "controller":
            self.Vx = .098
        else:
            self.Vx = .07
        
        self.circular_path_pub = rospy.Publisher('/circle_trajectory', Path, queue_size=5)
        self.circular_path = Path()
        self.circular_path.header.frame_id = global_frame  
        self.angle_step = 2 * math.pi / self.num_points
        self.generateCircularPath()
        self.circular_path_pub.publish(self.circular_path)
        self.rate = rospy.Rate(20)
        
        
        self.omobotPathArray = []
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = self.Vx
        self.cmd_vel.angular.z = self.Vx/self.radius
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

    def start(self):
        tempOmoArray = []
        while not rospy.is_shutdown():
            try:
                # Check if the transformation is available at the current time
                if self.tf_listener.canTransform(global_frame, base_frame, rospy.Time(0)):
                    (trans, rot) = self.tf_listener.lookupTransform(global_frame, base_frame, rospy.Time(0))
                    # print(trans, rot)
                    
                    pose = PoseStamped()
                    pose.header.frame_id = self.robot_path.header.frame_id
                    pose.header.stamp = self.robot_path.header.stamp
                    pose.pose.position.x = trans[0]+ .03
                    pose.pose.position.y = trans[1]
                    # pose.pose.orientation.x = rot[0] 
                    # pose.pose.orientation.y = rot[1] 
                    # pose.pose.orientation.z = rot[2] 
                    pose.pose.orientation.w = 1.0 
                    self.robot_path.poses.append(pose)
                    tempOmoArray.append([pose.pose.position.x, pose.pose.position.y])
                    
                    if len(self.robot_path.poses) > self.num_points*15:
                        if self.omobotPathArray == []:
                            self.omobotPathArray = tempOmoArray
                            offset =  self.radius*1.3
                            circularPathArray = [ [self.center_x + offset * math.cos(self.angle_step * ind) - .025, self.center_y + offset * math.sin(self.angle_step * ind) + offset] for ind in range(len(self.omobotPathArray))]
                            np.save("src/mobot_pkg/data/omobotPath_{}.npy".format(self.mode), np.array(self.omobotPathArray))
                            np.save("src/mobot_pkg/data/circularPath_{}.npy".format(self.mode), np.array(circularPathArray))
                            print("Data seved in {self.omobotPathArray} {circularPathArray}")
                            
                            os.system('echo y | rosnode cleanup')
                            try:
                                path_publisher.cmd_vel_pub.publish(Twist())
                                path_publisher.cmd_vel_pub.publish(Twist())
                            except:
                                pass
                            sys.exit(0)
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
            offset =  self.radius*1.3
            x = self.center_x + offset * math.cos(angle)
            y = self.center_y + offset * math.sin(angle)

            pose = PoseStamped()
            pose.header.frame_id = self.circular_path.header.frame_id
            pose.header.stamp = self.circular_path.header.stamp
            pose.pose.position.x = x - .025
            pose.pose.position.y = y + offset
            pose.pose.orientation.w = 1.0  # assuming no orientation change
            self.circular_path.poses.append(pose)
            
    def publishCmdVel(self):
        # pass
        self.cmd_vel_pub.publish(self.cmd_vel)
        
                
        

if __name__ == '__main__':
    global path_publisher
    path_publisher = PathPublisher()
    path_publisher.start()

