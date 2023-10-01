cmake_minimum_required(VERSION 3.0.2)
project(disturbing_points_detector)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    sensor_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES intrusion_judge
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(disturbing_points_detector_node src/disturbing_points_detector_node.cpp src/disturbing_points_detector.cpp)
add_dependencies(disturbing_points_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
    disturbing_points_detector_node
    ${catkin_LIBRARIES}
    )
