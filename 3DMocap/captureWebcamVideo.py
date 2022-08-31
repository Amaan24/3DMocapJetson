"""trt_yolo_10fps.py

This script performs real-time object detection with
TensorRT optimized YOLO engine at 10 fps using usb cam input.
"""

import argparse

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver

##

from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps

import datetime

WINDOW_NAME = 'HD Webcam Footage'


def parse_args():
    """Parse input arguments."""
    desc = ('Capture, save and display live camera video on Jetson Nano')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    args = parser.parse_args()
    return args

def disp_and_save(cam):
#def disp_and_save():
    """Capture images from camera and save to file

    # Arguments
      cam: the camera instance (video source).
    """ 

    #cap = cv2.VideoCapture(0)    

    filename = str(datetime.datetime.now()).replace(" ", "_")
    out = cv2.VideoWriter('training_vids/{}.avi'.format(filename),cv2.VideoWriter_fourcc('M','J','P','G'), 30, (1280,720))

    while True:
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            break
        img = cam.read()
       # img = cap.read()
        # Stop application if camera fails
        if img is None:
            break
        out.write(img)
        down_width = 320
        down_height = 180
        down_points = (down_width, down_height)
        #imgResized = cv2.resize(img, down_points, interpolation= cv2.INTER_LINEAR)
        #cv2.imshow(WINDOW_NAME, imgResized)
        cv2.imshow(WINDOW_NAME, img)
           
        key = cv2.waitKey(30)      
        if key == 27:  # ESC key: quit program
            out.release()
            break

def main():
    # Make sure all required arguments have been passed
    args = parse_args()

    # Initialise Camera object
    cam = Camera(args)
    if not cam.isOpened():
        raise SystemExit('ERROR: failed to open camera!')

    # Open display and begin looping. Loops until user presses ESC
    open_window(
        WINDOW_NAME, 'Camera TensorRT YOLO Demo',
        640, 360)
#        cam.img_width, cam.img_height)
    disp_and_save(cam)
    #disp_and_save()


    # Shutdown everything gracefully
    print("Shuttting down everything gracefully...")

    cam.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
