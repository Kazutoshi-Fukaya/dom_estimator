cmake_minimum_required(VERSION 3.0.2)
project(dom_estimator)

add_compile_options(-o2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    dom_estimator_msgs
    geometry_msgs
    multi_localizer_msgs
    nav_msgs
    object_color_detector_msgs
    object_detector_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

# OpenCV
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 3.0 QUIET)
    if(NOT OpenCV_FOUND)
        find_package(OpenCV 3.4.3 QUIET)
        if(NOT OpenCV_FOUND)
            message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
        endif()
    endif()
endif()

catkin_package(
    INCLUDE_DIRS include
    ${OpenCV_INCLUDE_DIRS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# DOM_LIBRARIES
add_library(Dom SHARED
    utils/database.cpp
    utils/objects.cpp
    utils/object.cpp
    utils/buffer_object.cpp
)

# Utils
add_library(Utils SHARED
    ros_utils/objects_data_subscribers.cpp
)

# ==================== Main ====================
# dom_estimator_node
add_executable(${PROJECT_NAME}_node
    src/dom_estimator.cpp
    src/dom_estimator_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    Dom
    Utils
)

# ==================== ROS Utils ====================
# image_connector
add_executable(images_connector ros_utils/images_connector.cpp)
target_link_libraries(images_connector 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# object_data_publisher
add_executable(object_data_publisher ros_utils/object_data_publisher.cpp)
target_link_libraries(object_data_publisher ${catkin_LIBRARIES})