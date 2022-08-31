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

##
import numpy as np

from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.yolo_with_plugins import TrtYOLO

import rospy
from std_msgs.msg import Int16

WINDOW_NAME = 'Yolo Object Tracking'


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

def kf_and_yolo(cam, trt_yolo, conf_th):

    """Capture images from camera and do object detection.

    # Arguments
      cam: the camera instance (video source).
      trt_yolo: the TRT YOLO object detector instance.
      conf_th: confidence/score threshold for object detection.
    """ 
    pub = rospy.Publisher('yolo_cmd', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    starttime = time.time()
    val = 0
    no_yolo_output = 0
    startup = True
    prev_center = 0
    diff = 0

    total_frames = 0
    missed_frames = 0

    while True:
        # Stops tracking if target not detected for 1 second (except on startup)
        if no_yolo_output > 10 and startup==False:
            # Reset counter so update state is run as soon as target is detected again
            print("No target detected. Tracking stopped.")
            val = 0
            prev_center = 0

            # Do object detection until target is detected. Then go back to tracking
            img = cam.read()

            # Stop application if camera fails
            if img is None:
                break
            #cv2.imshow(WINDOW_NAME, img)
            boxes, confs, clss = trt_yolo.detect(img, conf_th)
            if (boxes.size > 0):
                no_yolo_output = 0
                print("Target acquired. Tracking resumed.")
        # If target found
        else:
            if (val%3!=0):
                val+=1
            # Run yolo measurement at 10Hz
            else:
                val+=1
                if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
                    break
                
                # Stop application if camera fails
                img = cam.read()
                #if (val%6==0):
                #out.write(img)

                if img is None:
                    break
                
                t = time.time() 
                boxes, confs, clss = trt_yolo.detect(img, conf_th)
                elapsed = time.time() - t
                total_frames = total_frames + 1
                print('Inferencing time: {}'.format(elapsed))
                if (boxes.size > 0):
                    startup = False 
                    no_yolo_output = 0
                    
                    centers = [None] * len(boxes)
                    distances = [None] * len(boxes)
                    for i in range(len(boxes)):
                        centers[i] = int((boxes[i][0]+boxes[i][2])/2)
 #                       distances[i] = abs(centers[i] - prev_center)
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
                    pub.publish(diff) 

                else:
                    missed_frames = missed_frames + 1
                    no_yolo_output += 1

                cv2.imshow(WINDOW_NAME, img)         

        waitTime = (0.02 - ((time.time() - starttime) % 0.02))

        key = cv2.waitKey(int(waitTime*1000)+1)      
        if key == 27:  # ESC key: quit program
            break


def main():
    # Make sure all required arguments have been passed
    args = parse_args()
    if args.category_num <= 0:
        raise SystemExit('ERROR: bad category_num (%d)!' % args.category_num)
    if not os.path.isfile('yolo/%s.trt' % args.model):
        raise SystemExit('ERROR: file (yolo/%s.trt) not found!' % args.model)

    # Initialise Camera object
    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    # Initialise tensorRT yolo
    trt_yolo = TrtYOLO(args.model, args.category_num, args.letter_box)

    # Open display and begin looping. Loops until user presses ESC
    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        cam.img_width, cam.img_height)

    kf_and_yolo(cam, trt_yolo, 0.4)

    # Shutdown everything gracefully
    print("Shuttting down everything gracefully...")

    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
