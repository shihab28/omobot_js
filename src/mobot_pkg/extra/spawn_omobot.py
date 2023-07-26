#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
import tf
from geometry_msgs.msg import Pose
import json

pose_goal = Pose()
pose_goal.position.x = 0.0; pose_goal.position.y = 0.0;  pose_goal.position.z = 0.0
# init_pose = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
# pose_goal.orientation.x = init_pose[0]
# pose_goal.orientation.y = init_pose[0]
# pose_goal.orientation.z = init_pose[0]
# pose_goal.orientation.w = init_pose[0]

def spawn_robot_model():
    # Initialize ROS node
    rospy.init_node('spawn_robot_node')

    # Wait for the spawn_model service to become available

    print("Waiting For Service")
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    print("Service Connected")

    # Read the URDF file content
    with open('omobot.urdf', 'r') as file:
        robot_urdf = file.read()

    # Fill in the necessary parameters for the spawn_model service request
    request = SpawnModel()
    

    request.model_name = 'omobot'  # Name of the robot model in Gazebo
    request.model_xml = robot_urdf  # URDF content of the robot
    request.robot_namespace = '/'  # Namespace of the robot
    request.initial_pose = pose_goal
    request.reference_frame = 'world'

    args_ = [request.model_name, request.model_xml, request.robot_namespace, request.initial_pose, request.reference_frame]


    # print(request.__dict__)
    # print(type(request.model_name), type(request.model_xml), type(request.robot_namespace), type(request.initial_pose))
  

    # # Spawn the robot model
    try:
        response = spawn_model.call(model_name=request.model_name, model_xml=request.model_xml, robot_namespace=request.robot_namespace, initial_pose=request.initial_pose, reference_frame=request.reference_frame)
        rospy.loginfo('Robot model spawned successfully')
    except rospy.ServiceException as e:
        rospy.logerr('Failed to spawn robot model: ' + str(e))

if __name__ == '__main__':
    spawn_robot_model()