#!/usr/bin/env python2

import numpy as np
import os, cv2
from matplotlib import pyplot as plt
import tf.transformations as tft


plotTrajectoryS = False



# fig = plt.figure(figsize=(1, 1))
# ax = fig.add_subplot(111)

# x = circularPathControlled[:, 0]
# y = circularPathControlled[:, 1]
# ax.plot(x, y, 'g', label='expected path')

# x = omobotPathControlled[:, 0]
# y = omobotPathControlled[:, 1]
# ax.plot(x, y, 'b--', label='path with controller' )

# x = omobotPathNotControlled[:, 0]
# y = omobotPathNotControlled[:, 1]
# ax.plot(x, y, 'r.', label='path without controller')

# # ax.set_title('Robot Trajectory ')
# ax.set_xlabel('X cordinate', fontsize=20)
# ax.set_ylabel('Y cordinate', fontsize=20)
# ax.legend(loc='upper right', fontsize=20)
# ax.grid(True, color=(0.75, 0.75, 0.75), linestyle='--', linewidth=0.5, fontsize=16)

# plt.show()


resolution = .001
def find_index_of_coordinate(coordinates, target_coordinate):
    
    # target_coordinate = np.array(target_coordinate)
    # Check where both coordinates match
    matches = np.all(coordinates == target_coordinate, axis=1)
    
    # Find the index of the first match
    try:
        index = np.where(matches)[0][0]
    except IndexError:
        index = -1  # No match found
    
    return index

def find_closest_distances(array1, array2):
    diff = array1[:, np.newaxis, :] - array2[np.newaxis, :, :]
    dist_squared = np.sum(diff**2, axis=2)
    min_distances = np.sqrt(np.min(dist_squared, axis=1))
    
    return min_distances

def findDistance(array1, array2):
    diff = array1 - array2
    diff_sq = diff**2
    dist = np.sqrt(np.sum(diff_sq, axis=1))
    # for ind in range(3000, 3100):
    #     print(ind, array1[ind], array2[ind], diff[ind], diff_sq[ind], dist[ind])    
    return dist


def plotTrajectory() :
    with open ("src/mobot_pkg/data/circularPath_controller.npy", 'rb') as rf:
        circularPathControlled = (np.load(rf)/resolution).astype(np.int16)
    with open ("src/mobot_pkg/data/omobotPath_controller.npy", 'rb') as rf:
        omobotPathControlled = (np.load(rf)/resolution).astype(np.int16)
    with open ("src/mobot_pkg/data/omobotPath_noController.npy", 'rb') as rf:
        omobotPathNotControlled = (np.load(rf)/resolution).astype(np.int16)

    # distController = np.linalg.norm(omobotPathControlled - circularPathControlled)
    # distNoController = np.linalg.norm(omobotPathNotControlled - circularPathControlled)

    controllerCordinates = []

    for ind, xy in enumerate(omobotPathControlled):
        # print(ind, xy, circularPathControlled[ind])
        print(ind, xy, find_index_of_coordinate(circularPathControlled, xy))
        
    # print(find_closest_distances(omobotPathControlled, circularPathControlled))
    # print(distNoController)

    # plt.close('all')


def convertAngles(fileName, array):
    tempArray = []
    for vals in array:
        rot0 = tft.euler_from_quaternion([0.0, 0.0, vals[3], vals[4]])[2]
        rot1 = tft.euler_from_quaternion([0.0, 0.0, vals[7], vals[8]])[2]
        rot2 = tft.euler_from_quaternion([0.0, 0.0, vals[11], vals[12]])[2]
        tempArray.append( [vals[0], vals[1], vals[2],  rot0, vals[5], vals[6], rot1, vals[9], vals[10], rot2] )
    np.save(fileName, np.array(tempArray))
    return np.array(tempArray)
    


def plotTfMap():
    
    convertedFIle = "src/mobot_pkg/data/tfDataConv.npy"
    # with open ("src/mobot_pkg/data/tfData.npy", 'rb') as rf:
    #     tfMap = np.load(rf)
    # convertAngles(convertedFIle, tfMap)
    with open (convertedFIle, 'rb') as rf:
        tfMap = np.load(rf)
          
    tfData = tfMap[1324:-500, :]
    
    
    rows, cols = tfData.shape
    print(rows, cols) 
    
    # for ind, arr in enumerate(tfData):
    
    fig = plt.figure(figsize=(1, 1))
    ax = fig.add_subplot(111)
    
    
    
    x = [ind//50 for ind in range(rows)]
    
    y = findDistance(tfData[:, 1:3], tfData[:, 1:3])
    
    


    y = findDistance(tfData[:, 1:3], tfData[:, 4:6])
    print("findDistance odom")
    
    arrow_length = .2
    ax.plot(  x, y, 'b', label="Odometry's (/odom) origin" )
    
    

    y = findDistance(tfData[:, 1:3], tfData[:, 7:9])
    print("findDistance base")
    ax.plot(  x, y, 'r', label="Robot's (/base_footprint) origin" )
    
    y_cum = np.cumsum(y)/50
    x_cum = x[:len(y_cum)]
    ax.plot(  x_cum, y_cum, 'g', label="Total Distance Covered by the Robot" )
    
    # ax.set_title('Robot Trajectory ')
    ax.set_ylabel("Distance from Map's origin in XY plane (meter)", fontsize=20)
    ax.set_xlabel('Time Pass (seconds)', fontsize=20)
    ax.legend(loc='upper right', fontsize=20)
    ax.grid(True, color=(0.75, 0.75, 0.75), linestyle='--', linewidth=0.5)
            

    plt.show()

plotTfMap()