#!/usr/bin/env python2

'''
Owner Information
Author: Shihab Uddin Ahamad
Email: shihab.ahamad28@gmail.com
Date: 03/20/2024
Description: 

'''

import rospy, os, yaml
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from multiprocessing import Queue
from std_msgs.msg import Float32, Int16, String
from actionlib_msgs.msg import GoalID


refresh_rate = 10
mempath = "/dev/shm/"
if os.path.isdir("/dev/shm/") == False:
    os.makedirs(mempath)
fallFileLoc = "{}fallInfo.txt".format(mempath)

mapZonesCoordiantes = {
    "exploration":{
        "A"    : [   0.0,    0.0, 0.0,      0.0, 0.0,     0.0,     1.0],
        "B"    : [ 2.596, -1.943, 0.0,      0.0, 0.0,     1.0,     0.0],
        "C"    : [-1.226, -4.323, 0.0,      0.0, 0.0,   -.707,   0.707],
        "D"    : [-0.918, -6.999, 0.0,      0.0, 0.0,     0.0,     1.0],
        "E"    : [ 2.492, -6.384, 0.0,      0.0, 0.0,   0.707,   0.707],
        "F"    : [ 5.937, -6.994, 0.0,      0.0, 0.0,     1.0,     0.0],
        "G"    : [ 5.910, -0.355, 0.0,      0.0, 0.0,     1.0,     0.0],
        "H"    : [   0.0,    0.0, 0.0,      0.0, 0.0,     0.0,     1.0],
    },
    "dock" : {
        "H"    : [ 0.0, 0.0, 0.0, 0.0, 0.0,   0.0,   1.0],
        "C"    : [0.02, 0.5, 0.0, 0.0, 0.0, 0.708, 0.708]
    },
    "goal" : {
        "0"    : [ 0.0, 0.0, 0.0, 0.0, 0.0,   0.0,   1.0]
    }
}

with open("src/mobot_pkg/config/omobot.yaml", "r") as yf:
    try:
        omobotConfig = yaml.safe_load(yf)
        # print(omobotConfig)
    except yaml.YAMLError as exc:
        print(exc)

# docking_pose = omobotConfig["docking"]
docking_pose = mapZonesCoordiantes["dock"]["C"]
explorationPoints = list(mapZonesCoordiantes["exploration"].keys())



def uninitiateDocking(batStatePub):
    pass

def initiateNavigation(pub, mode="dock", LOC="H"):

    Loc = LOC.upper()
    mode = mode.lower()
    goal = PoseStamped()


    goal.header.seq = 3
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    loc_pose = mapZonesCoordiantes[mode][Loc]
    print("Publishing Goal data...........\n")
    print("dockingPose : ",   loc_pose)
    goal.pose.position.x =    loc_pose[0]
    goal.pose.position.y =    loc_pose[1]
    goal.pose.position.z =    loc_pose[2]
    goal.pose.orientation.x = loc_pose[3]
    goal.pose.orientation.y = loc_pose[4]
    goal.pose.orientation.z = loc_pose[5]
    goal.pose.orientation.w = loc_pose[6]

    pub.publish(goal)

    print("Published goal data...........\n")
    pass


def voltageFeedbackCB(datas, queue):
    queue.put(datas.data)


voltageSample = 10 #int(10 * refresh_rate)
voltageThreshold1 = 11.3
voltageThreshold2 = 11.4

startTime = 0.000000000000000 
pathLength = 10


def timePassed():
    global startTime
    return   round(float(rospy.get_time()) - startTime, 6)



def pathPlanCB(datas):
    global pathLength
    pathList = datas.poses
    pathLength = len(pathList)

    

def ExplorationNode(queue):
    global prevTimeWhl, prevTimePos, startTime

    rospy.init_node("ExplorationNode", anonymous=False)
    rospy.Subscriber("/battery_voltage", Float32, voltageFeedbackCB, callback_args=queue, queue_size=1)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, pathPlanCB,  queue_size=1)
    robotOpState = "exploration" # states = ["exploration", "teleoperation", "docking", "navigation"]

    docStatePub = rospy.Publisher("/dockingState", Int16, queue_size=1)
    robStatePub = rospy.Publisher("/robotOperationState", String, queue_size=1)


    goal_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=2)
    navPausePub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=2)
    navPausePub= rospy.Publisher("/cmd_vel_joy", Twist, queue_size=1)
    
    rate = rospy.Rate(refresh_rate)

    rate.sleep()
    totalVoltage = 0.0
    avgVoltage = 0.0
    samples = 0
    robState = String()
    robState.data = "None"
    joy_msg = Twist()
    holdState = False
    startTime = rospy.get_time() 
    pauseTime = timePassed()
    print("startTime : ", startTime)

    explorationPoints = ["A", "G", "B", "C", "D", "E", "F", "E", "B", "H"]
    
    curExpInd = -1
    readyForNextPoint = True
    nextPoint = None
    fallDetectionTime = timePassed()
    lastDockTime = timePassed()
    isDocked = False
    print("fallDetectionTime : ", fallDetectionTime - startTime)
    timeFromLastDetect = 0.0
    dockTImePass = 0.0
    while not rospy.is_shutdown():
        try:
            curV = queue.get(timeout=1/refresh_rate)
            totalVoltage += curV
            samples += 1

        except Exception as e:
            pass

        if os.path.exists(fallFileLoc):
            # navPausePub.publish(True)
            navPausePub.publish(joy_msg) 
            print("Fall Detected ..................")
            holdState = True
            if not holdState:
                fallDetectionTime = timePassed()

        else:
            print(samples, timePassed(), "pathLen : {}".format(pathLength), "lastDetOn: {}_{}".format(timeFromLastDetect, int(timeFromLastDetect) >= 30),  "robSt : {}".format(robState.data), "isDock: {}".format(isDocked), "ready4N_P: {}".format(readyForNextPoint), "avgV: {}".format(avgVoltage), "nxtP: {}".format(nextPoint), "dTimeP: {}".format(dockTImePass))
            
            timeFromLastDetect = timePassed() - fallDetectionTime
            if int(timeFromLastDetect) >= 20:
                
                holdState = False
                
                if robState.data == "None":
                    robState.data = "monitoring"

                if  samples >= voltageSample:
                    avgVoltage = totalVoltage/samples

                    if robState.data == "monitoring":
                        
                        print(robState.data, readyForNextPoint, pathLength)
                        if readyForNextPoint == True:
                            curExpInd += 1
                            if curExpInd < len(explorationPoints):
                                nextPoint = explorationPoints[curExpInd]
                                
                                rospy.sleep(1)
                                initiateNavigation(goal_publisher, "exploration", nextPoint)
                                rospy.sleep(1)
                                # initiateNavigation(goal_publisher, "exploration", nextPoint)
                            else:
                                robState.data = "docking"
                                nextPoint = "H"
                                rospy.sleep(1)
                                initiateNavigation(goal_publisher, "dock", nextPoint)
                                rospy.sleep(1)
                            
                            readyForNextPoint = False
                        else:
                            if pathLength <= 10:
                                readyForNextPoint = True
                            else:
                                readyForNextPoint = False
                    
                    elif robState.data == "docking":
                        if pathLength <= 8:
                            readyForNextPoint = True
                            robState.data = "charging"
                            lastDockTime = timePassed()
                            isDocked = True
                        else:
                            readyForNextPoint = False
                            isDocked = False
                    
                    elif robState.data == "charging":
                        
                        if isDocked and avgVoltage >=  voltageThreshold2 and (dockTImePass >= .5*20.0):
                            robState.data = "monitoring"
                            readyForNextPoint = True
                            curExpInd = -1
                            
                        elif not isDocked:
                            if pathLength <= 8:
                                readyForNextPoint = True
                                isDocked = True
                            else:

                                readyForNextPoint = False
                        else:
                            robState.data = "charging"
                            
                    dockTImePass = timePassed() - lastDockTime
                    totalVoltage = 0.0
                    samples = 0
                
            
                robStatePub.publish(robState)
                

        rate.sleep()

def startExplorationNode():
    queue = Queue()
    queue.put([0, 0, 0, 0])

    ExplorationNode(queue)

if __name__ == "__main__":
    
    startExplorationNode()
    
    
    
#!/usr/bin/env python2

'''
Owner Information:
Author: Shihab Uddin Ahamad
Email: shihab.ahamad28@gmail.com
Date: 03/20/2024

Description:
This script enables a robot to autonomously explore a series of predefined locations on a static map within a ROS (Robot Operating System) environment. It monitors the robot's battery state through the "/battery_voltage" topic and decides whether to continue exploring or return to a docking station for charging. Exploration waypoints and docking positions are predefined in the script and can be customized for any static map setup.

Key Features:
- Autonomous navigation through a list of predefined waypoints.
- Intelligent docking behavior based on battery level thresholds.
- Fall detection to pause navigation, ensuring safety.
- Utilization of shared memory ("/dev/shm/") for efficient data handling.
- Dynamic exploration path planning with feedback from the battery state.

Workflow:
1. The robot starts in exploration mode, navigating through predefined waypoints.
2. Battery voltage is periodically checked. If the voltage drops below a specified threshold, the robot navigates to the docking station.
3. The robot can detect falls, pausing its navigation to ensure safety.
4. Once docked and charged, or if no fall is detected, the robot resumes its exploration from the last point or restarts the cycle.
'''

import rospy, os, yaml
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from multiprocessing import Queue
from std_msgs.msg import Float32, Int16, String
from actionlib_msgs.msg import GoalID

# Initialization and global variables
refresh_rate = 10
mempath = "/dev/shm/"
if not os.path.isdir(mempath):
    os.makedirs(mempath)
fallFileLoc = f"{mempath}fallInfo.txt"

# Map zones coordinates for exploration and docking
mapZonesCoordinates = {
    "exploration": {...},
    "dock": {...},
    "goal": {...}
}

# Load robot configuration or map zones from a YAML file
with open("src/mobot_pkg/config/omobot.yaml", "r") as yf:
    try:
        omobotConfig = yaml.safe_load(yf)
    except yaml.YAMLError as exc:
        print(exc)

# Define functions for docking, navigation, and battery state handling
def uninitiateDocking(batStatePub):
    pass

def initiateNavigation(pub, mode="dock", LOC="H"):
    # Constructs and publishes navigation goals based on mode and location
    pass

def voltageFeedbackCB(data, queue):
    # Callback function for updating the battery voltage state
    pass

# Main class for exploration node
class ExplorationNode:
    def __init__(self):
        # Initialization of ROS node, subscribers, and publishers
        pass

    def run(self):
        # Main loop for exploration, docking decision-making, and fall detection handling
        pass

# Main function to start the exploration node
def startExplorationNode():
    queue = Queue()
    queue.put([0, 0, 0, 0])
    ExplorationNode(queue).run()

if __name__ == "__main__":
    startExplorationNode()

