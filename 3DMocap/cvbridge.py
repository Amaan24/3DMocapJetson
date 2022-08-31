#!/usr/bin/env python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import ros_numpy

from utils.display import open_window
WINDOW_NAME = 'Yolo Object Tracking'
# Instantiate CvBridge
bridge = CvBridge()

count = 0

def image_callback(msg):
    global count
    if (count%6 == 0):
        print("Received an image!")
        cv2_img = ros_numpy.numpify(msg)
        try:
            print("hi")
            # Convert your ROS Image message to OpenCV2
            
        except:
            print("Error")
        else:
            # Save your OpenCV2 image as a jpeg
            cv2.imshow(WINDOW_NAME, cv2_img)    
            cv2.waitKey(1) 
            count = 0
    else:
         print(count)

    count += 1

def main():
    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        540, 360)
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/video_source/raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
