cmake_minimum_required(VERSION 2.8.3)

project(encoder_odometry)

find_package(catkin REQUIRED COMPONENTS nav_msgs sensor_msgs tf2_ros)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
