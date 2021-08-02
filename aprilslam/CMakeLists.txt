cmake_minimum_required(VERSION 2.8.3)
project(aprilslam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "-std=c++11 -Ofast -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp image_transport cv_bridge image_geometry
    apriltag_mit std_msgs tf2_ros rosbag
    pcl_conversions
    pcl_ros
    )

find_package(cmake_modules)
find_package(GTSAM REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)
find_package(yaml-cpp REQUIRED QUIET)

add_message_files(DIRECTORY msg FILES Apriltag.msg Apriltags.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    )

# aprilslam library
add_library(${PROJECT_NAME}
    src/mapper/mapper.cpp
    src/mapper/tag_map.cpp
    src/utils/utils.cpp
    src/visualizer/visualizer.cpp
    src/detector/detector_node.cpp
    src/mapper/mapper_node.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    gtsam
    yaml-cpp
    )

# detector node
add_executable(detector_node
    src/detector/detector_main.cpp
    )
target_link_libraries(detector_node
    ${PROJECT_NAME}
    )

add_executable(mapper_node
    src/mapper/mapper_main.cpp
    )
target_link_libraries(mapper_node
    ${PROJECT_NAME}
    gtsam
    )

add_executable(zed_camera_info_pub 
src/utils/zed_camera_info_pub.cpp
)

target_link_libraries(zed_camera_info_pub
${PROJECT_NAME})

add_executable(bagtime_converter 
src/utils/bagtime_converter.cpp
)

target_link_libraries(bagtime_converter
${PROJECT_NAME})

add_executable(zed_image_splitter 
src/utils/zed_image_splitter.cpp
)

target_link_libraries(zed_image_splitter
${PROJECT_NAME})

add_executable(pcd_rviz_viewer 
src/utils/pcd_rviz_viewer.cpp
)

target_link_libraries(pcd_rviz_viewer
${PROJECT_NAME})

add_executable(tf_publisher 
src/utils/tf_publisher.cpp
)

target_link_libraries(tf_publisher
${PROJECT_NAME})

# dependencies
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gencpp
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    )
