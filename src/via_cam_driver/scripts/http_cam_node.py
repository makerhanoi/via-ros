#!/usr/bin/env python3
from __future__ import print_function

import sys
import urllib.request

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# TODO (vietanhdev): Use param here
CAM_URL = "http://192.168.4.1:81/stream"


class VIACamDriver:
    def __init__(self):
        self.__app_name = "via_cam_driver"

        self._cv_bridge = CvBridge()

        self._topic_name = rospy.get_param('~topic_name', '/camera/rgb')
        rospy.loginfo("[%s] (topic_name) Publishing Images to topic  %s",
                      self.__app_name, self._topic_name)

        self._image_publisher = rospy.Publisher(
            self._topic_name, Image, queue_size=3)

        self._rate = rospy.get_param('~publish_rate', 20)
        rospy.loginfo(
            "[%s] (publish_rate) Publish rate set to %s hz", self.__app_name, self._rate)

        self._frame_id = rospy.get_param('~frame_id', 'camera')
        rospy.loginfo("[%s] (frame_id) Frame ID set to  %s",
                      self.__app_name, self._frame_id)

        self.stream = urllib.request.urlopen(CAM_URL)
        self.buffer = bytes()

    def run(self):
        ros_rate = rospy.Rate(self._rate)

        try:
            while not rospy.is_shutdown():
                self.buffer += self.stream.read(1024)
                a = self.buffer.find(b'\xff\xd8')
                b = self.buffer.find(b'\xff\xd9')

                if a != -1 and b != -1:
                    cv_image = self.buffer[a:b+2]
                    self.buffer = self.buffer[b+2:]
                    # Decode image
                    cv_image = cv2.imdecode(np.frombuffer(
                        cv_image, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if cv_image is not None:
                        ros_msg = self._cv_bridge.cv2_to_imgmsg(
                            cv_image, 'bgr8')
                        ros_msg.header.frame_id = self._frame_id
                        ros_msg.header.stamp = rospy.Time.now()
                        self._image_publisher.publish(ros_msg)
                    ros_rate.sleep()
        except CvBridgeError as e:
            rospy.logerr(e)


def main(args):
    rospy.init_node('via_cam_driver', anonymous=True)

    image_publisher = VIACamDriver()
    image_publisher.run()


if __name__ == '__main__':
    main(sys.argv)
