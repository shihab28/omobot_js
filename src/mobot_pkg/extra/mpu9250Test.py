#!/usr/bin/env python2


import time
import numpy as np
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis



from imu.mpu9250 import MPU9250
imu = MPU9250(bus=1, device_addr=0x68)



def publish_temp(timer_event):
    temp_msg = Temperature()
    # temp_msg.header.frame_id = IMU_FRAME
    # temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
    # temp_msg.header.stamp = rospy.Time.now()
    # temp_pub.publish(temp_msg)

def publish_imu(timerEvent):


    accel_x = imu.accel.ixyz[0] / 16384.0
    accel_y = imu.accel.ixyz[1] / 16384.0
    accel_z = imu.accel.ixyz[2] / 16384.0
    accel = accel_x, accel_y, accel_z

    gyro_x = imu.gyro.ixyz[0] / 131.0
    gyro_y = imu.gyro.ixyz[1] / 131.0
    gyro_z = imu.gyro.ixyz[3] / 131.0
    gyro = gyro_x, gyro_y, gyro_z


    scale = 6.6666 
    mag = list(map(lambda x, y : x*y/scale, imu.mag.ixyz, imu.mag_correction))


    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    print(orientation, mag)

if __name__ == '__main__':
    rospy.init_node('imu_node')

    # bus = smbus.SMBus(rospy.get_param('~bus', 1))
    # ADDR = rospy.get_param('~device_address', 0x68)
    # if type(ADDR) == str:
    #     ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~base_link', 'imu1_link')

    # bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    temp_pub = rospy.Publisher('temperature', Temperature, queue_size=20)
    imu_pub = rospy.Publisher('imu/raw', Imu, queue_size=20)
    imu_timer = rospy.Timer(rospy.Duration(0.01), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    