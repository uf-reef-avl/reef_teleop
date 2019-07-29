cmake_minimum_required(VERSION 2.8.3)
project(reef_teleop)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    joy
    roscpp
    rospy
    sensor_msgs
    reef_msgs
    rosflight_msgs
)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS geometry_msgs joy roscpp rospy sensor_msgs reef_msgs
    DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
    reef_teleop     src/reef_teleop_node.cpp
    src/Teleop.cpp  include/Teleop.h
)
add_dependencies(reef_teleop reef_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS}  reef_msgs)
target_link_libraries(reef_teleop ${catkin_LIBRARIES})

install(TARGETS reef_teleop
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)