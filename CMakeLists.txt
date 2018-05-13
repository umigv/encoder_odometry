cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(encoder_odometry)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp
                                        sensor_msgs umigv_utilities)

find_package(Boost REQUIRED)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs umigv_utilities
               DEPENDS Boost)

include_directories(include/ ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(SOURCES src/twist_publisher.cpp)

add_executable(encoder_odometry_node src/encoder_odometry_node.cpp ${SOURCES})
target_link_libraries(encoder_odometry_node ${catkin_LIBRARIES}
                                            ${Boost_LIBRARIES})
add_dependencies(encoder_odometry_node ${catkin_EXPORTED_TARGETS})

install(TARGETS encoder_odometry_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
