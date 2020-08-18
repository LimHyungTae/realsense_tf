cmake_minimum_required(VERSION 2.8.3)
project(realsense_tf)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    cv_bridge
    message_generation
    laser_geometry
    pcl_ros
    )

add_message_files(
    # FILES
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
   )


find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES beginner_tutorials
    #  CATKIN_DEPENDS roscpp rospy std_msgs
    #  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    tools/rgtlib
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -msse3 ${CMAKE_CXX_FLAGS}")

# Conversion library
set(rgtlib_source
    tools/rgtlib/common.cpp
    tools/rgtlib/common.h
)


add_executable(viz_tf src/main.cpp)
add_dependencies(viz_tf realsense_tf_generate_messages_cpp)
target_link_libraries(viz_tf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

