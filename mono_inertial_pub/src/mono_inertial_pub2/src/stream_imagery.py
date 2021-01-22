#!/usr/bin/env python

## Publish vision data to topic /camera/image_raw 
## Vison data is in the form of a Image (mono8) Message

from picamera import PiCamera
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image 
import math
import time
import numpy as np


def stream_image_data(pub, camera):

    bridge = CvBridge()

    # adjusting to capture a single image and repeat production of it
    cam_data_obj = np.empty((640 * 640 * 3,), dtype=np.uint8)
    camera.capture(cam_data_obj, 'rgb')
    time_stamp = rospy.Time.now()
    img_msg = Image()
    img_msg.header.stamp = time_stamp
    img_msg.height = 640
    img_msg.width = 640
    img_msg.step = 1920
    img_msg.encoding = 'rgb8'
    img_msg.data = cam_data_obj.tolist()
    try:
        while not rospy.is_shutdown():
            try:
                # intialize data obj for image, needs to be flat, with vert pixels to nearest 16 and horiz to 
                # the nearest 32 (https://picamera.readthedocs.io/en/release-1.13/recipes2.html)
                # note that each image is roughly 1,228,800 bytes (or ~1.2 MB) .... will this work over a 1 Mbps connection??
                cam_data_obj = np.empty((640 * 640 * 3,), dtype=np.uint8)
                camera.capture(cam_data_obj, 'rgb')
                # TODO may need to figure out difference between this timing and the imu's rospy.Time.from_sec() way
                time_stamp = rospy.Time.now()
                img_msg = Image()
                img_msg.header.stamp = time_stamp
                img_msg.height = 640
                img_msg.width = 640
                img_msg.step = 1920
                img_msg.encoding = 'rgb8'
                img_msg.data = cam_data_obj.tolist()

                rospy.loginfo('image taken and published')
                #rospy.loginfo(img_msg)
                pub.publish(img_msg)
                #cam_rate = rospy.Rate(10)#20)
                #cam_rate.sleep()
                #time.sleep(5)
            except CvBridgeError as e:
                rospy.loginfo(e)
    except KeyboardInterrupt:
        raise KeyboardInterrupt
        return

if __name__ == '__main__':
    pub0 = rospy.Publisher('/camera/image_raw', Image, queue_size=1000)
    rospy.init_node('stream_imagery', anonymous=True)   

    # initialize the camera
    camera = PiCamera(resolution=(640, 640))
    # iso values of 100 or 200 good for daytime, and 400 or 800 for low light
    camera.iso = 200

    # finish camera calib (needed to wait after setting iso value)
    time.sleep(5)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g

    stream_image_data(pub0, camera)
