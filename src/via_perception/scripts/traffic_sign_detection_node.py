#!/usr/bin/env python3
from __future__ import print_function

import _thread
import os
import sys
import time
import urllib.request
from os import listdir
from os.path import isfile, join

import cv2
import numpy as np
import roslib
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('via_perception'),
                "scripts", "traffic_sign_detection", "yolov5"))

from traffic_sign_detection.traffic_sign_detector import TrafficSignDetector


class TrafficSignDetectionNode:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._topic_name = rospy.get_param('~topic_name', '/camera/rgb')
        self._image_subscriber = rospy.Subscriber(
            self._topic_name, Image, self.image_callback, queue_size=1)
        self._rate = 20
        self._bridge = CvBridge()
        self._model = None

        self._model_path = os.path.join(rospack.get_path(
            'via_perception'), "models", "via_traffic_sign_detection_20210321.pt")
        self._model = TrafficSignDetector(self._model_path, use_gpu=False)

    def image_callback(self, msg):
        try:
            # Wait for model
            while self._model is None:
                time.sleep(1)
            # Convert your ROS Image message to OpenCV2
            cv2_img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            preds, viz_img = self._model.predict(cv2_img, visualize=True)
            cv2.imshow("Traffic signs", viz_img)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def run(self):
        ros_rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            ros_rate.sleep()


def main(args):
    rospy.init_node('lane_detection_node', anonymous=True)
    image_subscriber = TrafficSignDetectionNode()
    image_subscriber.run()


if __name__ == '__main__':
    main(sys.argv)
