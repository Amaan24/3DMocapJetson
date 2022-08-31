#!/usr/bin/env python3
"""trt_yolo_10fps.py

This script performs real-time object detection with
TensorRT optimized YOLO engine at 10 fps using usb cam input.
"""

import os
import time
import argparse

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver
import pycuda.driver as cuda

##
import numpy as np

from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.yolo_with_plugins import TrtYOLO

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
import ros_numpy

from queue import Queue

WINDOW_NAME = 'Yolo Object Tracking'

prev_center = 0
diff = 0
count = 0

q = Queue(maxsize = 3)

def parse_args():
    """Parse input arguments."""
    desc = ('Capture and display live camera video, while doing '
            'real-time object detection with TensorRT optimized '
            'YOLO model on Jetson')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument(
        '-c', '--category_num', type=int, default=80,
        help='number of object categories [80]')
    parser.add_argument(
        '-m', '--model', type=str, required=True,
        help=('[yolov3-tiny|yolov3|yolov3-spp|yolov4-tiny|yolov4|'
              'yolov4-csp|yolov4x-mish]-[{dimension}], where '
              '{dimension} could be either a single number (e.g. '
              '288, 416, 608) or 2 numbers, WxH (e.g. 416x256)'))
    parser.add_argument(
        '-l', '--letter_box', action='store_true',
        help='inference with letterboxed image [False]')
    args = parser.parse_args()
    return args


class yolo:
    def __init__(self):
        self.ctx = cuda.Device(0).make_context()
        self.trt_yolo = TrtYOLO('yolov4-tiny-obj_person')
        self.pub = rospy.Publisher('yolo_cmd', Int16, queue_size=10)
        self.sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.count = 0

    def image_callback(self, msg):
        if (self.count%10 == 0):          
            img = ros_numpy.numpify(msg)
            self.ctx.push()
            boxes, confs, clss = self.trt_yolo.detect(img, 0.4)
            self.ctx.pop()

            if (boxes.size > 0):
                centers = [None] * len(boxes)
                distances = [None] * len(boxes)
                for i in range(len(boxes)):
                    centers[i] = int((boxes[i][0]+boxes[i][2])/2)
                    #distances[i] = abs(centers[i] - prev_center)
                    distances[i] = abs(centers[i] - 640)
                #print(distances)
                index_min = min(range(len(distances)), key=distances.__getitem__)
                #prev_center = centers[index_min]
                
                cv2.rectangle(img, (boxes[index_min][0],boxes[index_min][1]),(boxes[index_min][2],boxes[index_min][3]),(255,0,0),2)
                #center = int((boxes[0][0]+boxes[0][2])/2)
                center = centers[index_min]
                print('Center: {}'.format(center))
                
                # Calculate how far subject's COM is from center of image
                diff = 640 - center 
                diff_str = str(diff) + '\n'
            
                rospy.loginfo(diff)  
                self.pub.publish(diff) 

            cv2.imshow(WINDOW_NAME, img)
            cv2.waitKey(1)
            self.count = 0  

        self.count += 1 

def main():
    # Make sure all required arguments have been passed
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)

    # Open display and begin looping. Loops until user presses ESC
    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        640, 360)

    # Initialise tensorRT yolo
    #trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    # Initialise ROS Stuff
    rospy.init_node('talker_listener', anonymous=True)
    yolo()    
    rospy.spin()

    # Shutdown everything gracefully
    print("Shuttting down everything gracefully...")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
