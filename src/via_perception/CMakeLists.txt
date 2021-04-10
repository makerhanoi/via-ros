cmake_minimum_required(VERSION 2.8.3)
project(via_perception)

find_package(catkin REQUIRED
        cv_bridge
        sensor_msgs
        rospy
        std_msgs
        )

catkin_python_setup()

catkin_package()

install(PROGRAMS
        scripts/lane_detection_node.py
        scripts/traffic_sign_detection_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )