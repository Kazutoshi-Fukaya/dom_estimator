cmake_minimum_required(VERSION 3.0.2)
project(dom_estimator)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    multi_robot_msgs
    nav_msgs
    object_color_detector_msgs
    object_detector_msgs
    roscpp
    rospy
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node src/dom_estimator_node.cpp)

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

