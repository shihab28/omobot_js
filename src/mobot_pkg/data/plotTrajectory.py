#!/usr/bin/env python2

import numpy as np
import os, cv2
from matplotlib import pyplot as plt



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
    for ind in range(3000, 3100):
        print(ind, array1[ind], array2[ind], diff[ind], diff_sq[ind], dist[ind])

    
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

def plotTfMap():
    with open ("src/mobot_pkg/data/tfData.npy", 'rb') as rf:
        tfMap = np.load(rf)
    tfData = tfMap[1324:-500, :]
    
    
    rows, cols = tfData.shape
    print(rows, cols) 
    
    # for ind, arr in enumerate(tfData):
    
    fig = plt.figure(figsize=(1, 1))
    ax = fig.add_subplot(111)
    
    
    
    x = [ind//50 for ind in range(rows)]
    
    y = findDistance(tfData[:, 1:3], tfData[:, 1:3])
    
    print(y.shape)
    ax.plot(x, y, 'g', label='x,y cordinate of map')


    y = findDistance(tfData[:, 1:3], tfData[:, 5:7])
    ax.plot(x, y, 'b--', label='x,y cordinate of odom' )

    y = findDistance(tfData[:, 1:3], tfData[:, 9:11])
    ax.plot(x, y, 'r.', label='x,y cordinate of base_footprint')
    
    # ax.set_title('Robot Trajectory ')
    ax.set_xlabel("Distance from Map's origin (meter)", fontsize=20)
    ax.set_ylabel('', fontsize=20)
    ax.legend(loc='upper right', fontsize=20)
    ax.grid(True, color=(0.75, 0.75, 0.75), linestyle='--', linewidth=0.5)
            
        
    plt.show()

plotTfMap()