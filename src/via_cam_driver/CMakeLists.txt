cmake_minimum_required(VERSION 2.8.3)
project(via_cam_driver)

find_package(catkin REQUIRED
        cv_bridge
        sensor_msgs
        rospy
        std_msgs
        )

catkin_python_setup()

catkin_package()

install(PROGRAMS
        scripts/http_cam_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        launch/http_cam.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )