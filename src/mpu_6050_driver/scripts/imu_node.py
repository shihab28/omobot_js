#!/usr/bin/env python

import time
import smbus
import struct
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis, euler_from_quaternion, quaternion_from_euler
from mpu_6050_driver.registers import PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, TEMP_H,\
    GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H

# import mpu_6050_driver

ADDR = None
bus = None
IMU_FRAME = None

# from src.mpu6050.src import Kalman_EKF

# Define Kalman filter parameters
Q = 0.5  # Process noise covariance
R = 0.2   # Measurement noise covariance
# Initial state (orientation) estimation
initial_orientation = np.array([0.0, 0.0, 0.0])  # Roll, Pitch, Yaw
initial_estimate_error = np.eye(3)

# Initialize Kalman filter variables
x_hat = initial_orientation
P = initial_estimate_error

# IMU measurement function
def h(x):
    return x

# Kalman filter prediction step
def predict(x, P):
    x_predicted = x
    P_predicted = P + Q
    return x_predicted, P_predicted

# Kalman filter update step
def update(x_predicted, P_predicted, z):
    K = P_predicted / (P_predicted + R)
    x_updated = x_predicted + K * (z - x_predicted)
    P_updated = (1 - K) * P_predicted
    return x_updated, P_updated

# Simulate IMU measurements (replace this with actual IMU data)





# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
curAxis = 'x'
prevVal= {
    'x':[0,0],
    'y':[0,0],
    'z':[0,0]
}

def read_word(adr):

    global curAxis, prevVal
    # print("ADDR , addr : ", ADDR, adr)
    try:
        high = bus.read_byte_data(ADDR, adr)
        low = bus.read_byte_data(ADDR, adr+1)
        prevVal[curAxis][0] = high
        prevVal[curAxis][1] = low
    except:
        high =  prevVal[curAxis][0]
        low = prevVal[curAxis][1]

    val = (high << 8) + low
    
    return val

def read_word_2c(adr):
    val = read_word(adr)
    # print("val : ", val)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
    temp_msg.header.stamp = rospy.Time.now()
    temp_pub.publish(temp_msg)

cnt = 0
orientation_list = []
def publish_imu(timer_event):
    global curAxis
    global x_hat, P, Q, R, cnt, orientation_list
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME
    sample_numer = 10
    # Read the acceleration vals
    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0
    
    # Calculate a quaternion representing the orientation
    accel = accel_x, accel_y, accel_z
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    # Read the gyro vals
    try:
        gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
        gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
        gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0
        
        # Load up the IMU message
        o = imu_msg.orientation

    
        imu_msg.linear_acceleration.x = -accel_x
        imu_msg.linear_acceleration.y = -accel_y
        imu_msg.linear_acceleration.z = accel_z

        imu_msg.angular_velocity.x = -gyro_x
        imu_msg.angular_velocity.y = -gyro_y
        imu_msg.angular_velocity.z = gyro_z

        imu_msg.header.stamp = rospy.Time.now()
        measured_orientation = euler_from_quaternion(orientation)

        orientation_list.append(measured_orientation)
        cnt += 1
        if cnt >= sample_numer:
            for orient in orientation_list:
                x_hat_predicted, P_predicted = predict(x_hat, P)
                x_hat, P = update(x_hat_predicted, P_predicted, orient)
            
            [o.x, o.y, o.z, o.w] = orientation
            orientation = quaternion_from_euler(*x_hat[1].tolist())
            imu_pub.publish(imu_msg)
            print("orientation : ", orientation)
            cnt = 0
            orientation_list = []
    except:
        pass
    

    



temp_pub = None
imu_pub = None
if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    # print("IMU ADDR : ", ADDR)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~base_link', 'imu0_link')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    temp_pub = rospy.Publisher('temperature', Temperature, queue_size=10)
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    imu_timer = rospy.Timer(rospy.Duration(0.002), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.spin()
