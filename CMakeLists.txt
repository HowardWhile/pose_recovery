cmake_minimum_required(VERSION 2.8.3)
project(pose_recovery_pkg)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    roscpp
    rospy
    std_msgs
    tf
    cv_bridge
    )
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/pose_recovery_parameters.cfg
    )

catkin_package(
    )

include_directories(
    #include
    ${catkin_INCLUDE_DIRS}
    )
	
add_executable(pose_recovery_node src/pose_recovery_node.cpp)

target_link_libraries(pose_recovery_node
    pose_recovery_lib
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_library(pose_recovery_lib
    src/pose_recovery.cpp    
    )

add_dependencies(pose_recovery_lib
    ${PROJECT_NAME}_gencfg
    )




