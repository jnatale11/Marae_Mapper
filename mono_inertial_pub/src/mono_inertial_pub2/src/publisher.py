#!/usr/bin/env python

## Publish vision and motion data to topics /camera/image_raw and /imu
## Vison data is in the form of a Image (mono8) Message, and IMU in the form of an Imu Message 

import BNO055
from picamera import PiCamera
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image, Imu
from functools import partial
import threading
import math
from multiprocessing.pool import Pool
from contextlib import closing
import time
import numpy as np


def stream_imu_data(pub, sensor):
    # initialize empty covariance vector 
    cov_vector = [0] * 9

    try:
        while not rospy.is_shutdown(): 
            # Obtain all relevant data from Imu 
            # Orientation as a quaternion:
            orientation = sensor.read_quaternion() #x, y, z, w 
            # Gyroscope data (in degrees per second):
            ang_vel = sensor.read_gyroscope() # x, y, z
            cur_time = time.time() 
            # Accelerometer data without gravity (in meters per second squared)
            lin_accel = sensor.read_linear_acceleration() #x, y, z

            # Build ROS Imu Message Object
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

            #rospy.loginfo('imu_msg {}'.format(cur_time))
            #rospy.loginfo(imu_msg) 
            #rospy.loginfo('imu status: {}'.format(sensor.get_calibration_status()[0])) 
            pub.publish(imu_msg)
            imu_rate = rospy.Rate(100)#200) #this was 1k)
            imu_rate.sleep()
            #time.sleep(5)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
        return

def stream_image_data(pub, camera):

    bridge = CvBridge()
    try:
        while not rospy.is_shutdown():
            try:
                # intialize data obj for image, needs to be flat, with vert pixels to nearest 16 and horiz to 
                # the nearest 32 (https://picamera.readthedocs.io/en/release-1.13/recipes2.html)
                #cam_data_obj = np.empty((640 * 640 * 3,), dtype=np.uint8)
                #camera.capture(cam_data_obj, 'rgb')
                # TODO may need to figure out difference between this timing and the imu's rospy.Time.from_sec() way
                time_stamp = rospy.Time.now()
                '''
                img_msg = Image()
                img_msg.header.stamp = time_stamp
                img_msg.height = 640
                img_msg.width = 640
                img_msg.step = 1920
                img_msg.encoding = 'rgb8'
                img_msg.data = cam_data_obj.tolist()
                '''
                rospy.loginfo('image_taken {}'.format(time_stamp))
                #rospy.loginfo(img_msg)
                #pub.publish(img_msg)
                cam_rate = rospy.Rate(10)#20)
                cam_rate.sleep()
                #time.sleep(5)
            except CvBridgeError as e:
                rospy.loginfo(e)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
        return

if __name__ == '__main__':
    try:
        pub0 = rospy.Publisher('/camera/image_raw', Image, queue_size=1000)
        pub1 = rospy.Publisher('/imu', Imu, queue_size=10000)  
        rospy.init_node('publisher', anonymous=True)   

        # initialize the camera
        camera = PiCamera(resolution=(640, 640))
        # iso values of 100 or 200 good for daytime, and 400 or 800 for low light
        camera.iso = 200

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

        # finish camera calib (needed to wait after setting iso value)
        camera.shutter_speed = camera.exposure_speed
        camera.exposure_mode = 'off'
        g = camera.awb_gains
        camera.awb_mode = 'off'
        camera.awb_gains = g

        imu_stream = partial(stream_imu_data, pub1, bno)
        image_stream = partial(stream_image_data, pub0, camera)
        
        with closing(Pool(2)) as p:
            p.apply_async(imu_stream)
            p.apply_async(image_stream)
            p.terminate()
 
    except rospy.ROSInterruptException or KeyboardInterrupt:
        p.terminate()
        pass
