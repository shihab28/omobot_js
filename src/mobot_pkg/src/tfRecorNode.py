#!/usr/bin/env python2

import rospy, time, os, sys, signal
from geometry_msgs.msg import Twist
import math
import tf
import tf2_msgs
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np

def signal_handler(sig, frame):
    global path_publisher, fullTFArray
    np.save("src/mobot_pkg/data/tfData.npy", np.array(fullTFArray))
    print('You pressed Ctrl+C!')
    os.system('echo y | rosnode cleanup')
    try:
        path_publisher.cmd_vel_pub.publish(Twist())
        path_publisher.cmd_vel_pub.publish(Twist())
    except:
        pass
    sys.exit(0)
    
    
#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

        
        
            
sample = 0

foundFrames = [None, None, None]
tfArrayList = [0,     0, 0, 0, 0,    0, 0, 0, 0,    0, 0, 0, 0]
prevTfArrayList = tfArrayList
startTime = None
fullTFArray = []
def callback(msg, args):
    global sample, foundFrames, tfArrayList, prevTfArrayList, startTime, fullTFArray
    broadcaster, frames, pub, tf_buffer, listener = args
    # foundAllFrames = False

    
    index = 0
    for transform in msg.transforms:
        frame_ = transform.header.frame_id
        if (frame_ in frames):
            
            try:
                transform_base = transform_base = tf_buffer.lookup_transform('map', frame_, rospy.Time())
                
                
                
                if frame_ not in foundFrames:
                    
                    index = frames.index(frame_)
                    tfArrayList[0] = sample
                    tfArrayList[index*4 + 1:index*4 + 5] = transform_base.transform.translation.x, transform_base.transform.translation.y, transform_base.transform.rotation.z, transform_base.transform.rotation.w
                    foundFrames[index] = frame_
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn("Error getting transform: %s" % e)

            frame__ = transform.child_frame_id
            if frame__ in frames and frame__ == 'base_footprint':
                try:
                    transform_base = tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time())
                    
                    if frame__ not in foundFrames:
                        if startTime == None:
                            startTime = transform_base.header.stamp.secs
                        tfArrayList[0] = transform_base.header.stamp.secs - startTime
                        index = frames.index(frame__)
                        tfArrayList[index*4 + 1:index*4 + 5] = transform_base.transform.translation.x, transform_base.transform.translation.y, transform_base.transform.rotation.z, transform_base.transform.rotation.w
                        foundFrames[index] = frame__
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn("Error getting transform: %s" % e)



         
    # print(foundFrames)
    
    if len(foundFrames) == len(frames) and None not in foundFrames:
        sample += 1
        
        foundFrames = [None, None, None]
        
        # print(tfArrayList)
        if tfArrayList[0] == 0 and fullTFArray == []:
            array_msg = Float32MultiArray()
            array_msg.data = tfArrayList
            pub.publish(array_msg)
            prevTfArrayList = tfArrayList
            # print
            # print("##################") 
            print(tfArrayList)
            # print("###########################")
            tfArrayList = [0,     0, 0, 0, 0,    0, 0, 0, 0,    0, 0, 0, 0]
            fullTFArray.append(tfArrayList)
            
            
        elif tfArrayList[0] != prevTfArrayList[0]:
            array_msg = Float32MultiArray()
            array_msg.data = tfArrayList
            pub.publish(array_msg)
            prevTfArrayList = tfArrayList
            # print
            # print("##################") 
            print(tfArrayList)
            # print("###########################")
            tfArrayList = [0,     0, 0, 0, 0,    0, 0, 0, 0,    0, 0, 0, 0]
            fullTFArray.append(tfArrayList)
            
            
        
        
        
        


def main():
    rospy.init_node('tf_filter')
    frames = ['map', 'odom', 'base_footprint']#
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    arrayPub = rospy.Publisher('/tfArray', Float32MultiArray, queue_size=5)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, callback, (tf_broadcaster, frames, arrayPub, tf_buffer, listener))
    rospy.Subscriber('/tf_static', tf2_msgs.msg.TFMessage, callback, (tf_broadcaster, frames, arrayPub, tf_buffer, listener))
    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin()

if __name__ == '__main__':
    main()

