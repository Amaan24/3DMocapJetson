#! /bin/bash

v4l2-ctl --device=/dev/video1 --set-ctrl=exposure_auto_priority=1
v4l2-ctl --device=/dev/video1 --set-ctrl=exposure_auto=3

