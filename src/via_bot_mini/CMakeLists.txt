cmake_minimum_required(VERSION 2.8.3)
project(via_bot_mini)

find_package(catkin REQUIRED
        cv_bridge
        sensor_msgs
        rospy
        std_msgs
        )


catkin_package()

# install(PROGRAMS
#         scripts/via_bot_mini_node.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )

# install(FILES
#         launch/via_bot_mini.launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#         )