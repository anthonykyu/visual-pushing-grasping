#!/usr/bin/env python

import time
import matplotlib.pyplot as plt
from camera import Camera
import rospy

rospy.init_node('camera_test')
camera = Camera()
time.sleep(1) # Give camera some time to load data

while True:
    print('here')
    color_img, depth_img = camera.get_data()
    plt.subplot(211)
    plt.imshow(color_img)
    plt.subplot(212)
    plt.imshow(depth_img)
    plt.show()
