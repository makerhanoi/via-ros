#!/usr/bin/env python3
from __future__ import print_function

import sys
import time

import socket
import rospy

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

# TODO (vietanhdev): Use params here
CONTROL_IP = "192.168.4.1"
CONTROL_PORT = 1234

MAX_ANGLE = 1
MAX_THROTTLE = 0.8
sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
sk.settimeout(3000)
is_stopped = False

class VIABotMiniChassis:
    def __init__(self):
        self._cv_bridge = CvBridge()
        self.steering_sub = rospy.Subscriber("/set_steering", Float32, self.control_callback, queue_size=1)
        self._rate = 30

    def control_callback(self, steering):
        self.send_control(steering)
    
    def send_control(self, steering):
        # print(steering)
        steering = float(steering.data)

        right_motor_speed = left_motor_speed = 0.8

        steering *= 0.1
        steering = min(0.2, max(-0.2, steering))

        right_motor_speed += steering
        left_motor_speed -= steering

        right_motor_speed = -right_motor_speed

        print("r {} l {}".format(right_motor_speed, left_motor_speed))

        control_msg = "{} {}".format(
            left_motor_speed, right_motor_speed).encode('ascii')
        sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))

        self.last_control_time = time.time()


    def run(self):
        ros_rate = rospy.Rate(self._rate)
        try:
            while not rospy.is_shutdown():
                ros_rate.sleep()

            # Stop motors on shutdown 
            control_msg = "{} {}".format(
                0, 0).encode('ascii')
            sk.sendto(control_msg, (CONTROL_IP, CONTROL_PORT))
        except CvBridgeError as e:
            rospy.logerr(e)


def main(args):
    rospy.init_node('via_bot_mini_chassis_node', anonymous=True)
    viabot_chassis = VIABotMiniChassis()
    viabot_chassis.run()


if __name__ == '__main__':
    main(sys.argv)