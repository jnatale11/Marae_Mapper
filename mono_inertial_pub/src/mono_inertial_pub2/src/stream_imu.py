#!/usr/bin/env python

## Publish motion data to topic /imu
## IMU in the form of an Imu Message 

import BNO055

import rospy
from sensor_msgs.msg import Imu
import math
import time
import numpy as np


def stream_imu_data(pub, sensor):
    # initialize empty covariance vector 
    cov_vector = [0] * 9

    try:
        while True:
            print(time.time())
        while True:#not rospy.is_shutdown(): 
            # Obtain all relevant data from Imu 
            # Orientation as a quaternion:
            #orientation = sensor.read_quaternion() #x, y, z, w 
            # Gyroscope data (in degrees per second):
            #ang_vel = sensor.read_gyroscope() # x, y, z
            #cur_time = time.time() 
            # Accelerometer data without gravity (in meters per second squared)
            #lin_accel = sensor.read_linear_acceleration() #x, y, z

            # Build ROS Imu Message Object
            '''
            imu_msg = Imu() 
            # convert ang velocity to rad/s
            ang_vel = [vel * math.pi / 180 for vel in ang_vel]       
 
            imu_msg.header.stamp = rospy.Time.from_sec(cur_time) 
            [setattr(imu_msg.linear_acceleration, axis, lin_accel[i]) for i, axis in enumerate('xyz')]
            [setattr(imu_msg.angular_velocity, axis, ang_vel[i]) for i, axis in enumerate('xyz')] 
            [setattr(imu_msg.orientation, axis, orientation[i]) for i, axis in enumerate('xyzw')]
            imu_msg.orientation_covariance = cov_vector
            imu_msg.angular_velocity_covariance = cov_vector
            imu_msg.linear_acceleration_covariance = cov_vector 

            '''
            rospy.loginfo('imu_msg created')
            #rospy.loginfo(imu_msg) 
            #rospy.loginfo('imu status: {}'.format(sensor.get_calibration_status()[0])) 
            #pub.publish(imu_msg)
            #imu_rate = rospy.Rate(100)#200) #this was 1k)
            #imu_rate.sleep()
            #time.sleep(5)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
        return


if __name__ == '__main__':
    pub1 = rospy.Publisher('/imu', Imu, queue_size=10000)  
    rospy.init_node('stream_imu', anonymous=True)   

    # initialize the IMU 
    bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
    status, self_test, error = bno.get_system_status()
    if status == 0x01:
        print('System error: {0}'.format(error))
        print('See datasheet section 4.3.59 for the meaning.')
        raise rospy.exceptions.ROSInterruptException("IMU not in good health")

    # do not start publishing until IMU sensor is calibrated
    sys, gyro, accel, mag = bno.get_calibration_status()
    while sys != 3 and not rospy.is_shutdown():
        print('Imu Not Yet Calibrated... Status Sys:{system} Gyro:{gyro} Accel:{accel} Mag:{mag}'.format(system=sys, gyro=gyro, accel=accel, mag=mag))
        time.sleep(3)
        sys, gyro, accel, mag = bno.get_calibration_status()

    stream_imu_data(pub1, bno) 
