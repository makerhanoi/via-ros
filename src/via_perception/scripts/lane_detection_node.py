#!/usr/bin/env python3
from __future__ import print_function

import os
import sys

import cv2
import numpy as np
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from control import calculate_control_signal
from lane_detection.lane_detector import LaneDetector
from lane_detection.segmentation_model import LaneLineSegmentationModel

rospack = rospkg.RosPack()

class LaneDetectionNode:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._topic_name = rospy.get_param('~topic_name', '/camera/rgb')
        self._image_subscriber = rospy.Subscriber(
            self._topic_name, Image, self.image_callback, queue_size=1)
        self._rate = rospy.get_param('~lane_detection_rate', 20)
        self._bridge = CvBridge()

        self.model_path = os.path.join(rospack.get_path(
            'via_perception'), "models", "goodgame_laneline_pspnet_1712_0055.onnx")
        self.lane_line_model = LaneLineSegmentationModel(
            self.model_path, use_gpu=False)
        self.lane_detector = LaneDetector(
            use_deep_learning=True, lane_segmentation_model=self.lane_line_model)

        self._steering_publisher = rospy.Publisher(
            "/set_steering", Float32, queue_size=1)

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("cv2_img", cv2_img)
            cv2.waitKey(1)
            # Analyze image
            left_point, right_point, im_center, lane_viz = self.lane_detector.find_lane_lines(
                cv2_img, draw=True)
            cv2.imshow("lane_viz", lane_viz)
            cv2.waitKey(1)
            # Calculate speed and steering angle
            throttle, steering_angle = calculate_control_signal(
                left_point, right_point, im_center)
            self._steering_publisher.publish(steering_angle)
        except CvBridgeError as e:
            print(e)

    def run(self):
        ros_rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            ros_rate.sleep()


def main(args):
    rospy.init_node('lane_detection_node', anonymous=True)
    image_publisher = LaneDetectionNode()
    image_publisher.run()


if __name__ == '__main__':
    main(sys.argv)
