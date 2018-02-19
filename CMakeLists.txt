cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(encoder_odometry)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs sensor_msgs
                                        umigv_utilities tf2 tf2_geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(encoder_odometry_node src/encoder_odometry_node.cpp
                                     src/callback_handler.cpp
                                     src/base_state.cpp)
target_link_libraries(encoder_odometry_node ${catkin_LIBRARIES})
add_dependencies(encoder_odometry_node ${catkin_EXPORTED_TARGETS})

install(TARGETS encoder_odometry_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
