cmake_minimum_required(VERSION 3.5)
project(rvio2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Set C++ standard (C++11 is used here, adjust if needed)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable optimization and debug info
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g3")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)

# Include directories for headers
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

# Build library
add_library(${PROJECT_NAME} SHARED
  src/rvio2/System.cc
  src/rvio2/Ransac.cc
  src/rvio2/Tracker.cc
  src/rvio2/Updater.cc
  src/rvio2/Propagator.cc
  src/rvio2/InputBuffer.cc
  src/rvio2/Feature.cc
  src/rvio2/FeatureDetector.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  visualization_msgs
  tf2_ros
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  Eigen3
  OpenCV
)

# ament_export_libraries(${PROJECT_NAME})

# Build executables
add_executable(rvio2_mono src/rvio2_mono.cc)
ament_target_dependencies(rvio2_mono
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  visualization_msgs
  tf2_ros
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  Eigen3
  OpenCV
)
target_link_libraries(rvio2_mono ${PROJECT_NAME})

option(BUILD_RVIO2_MONO_EVAL "Build rvio2_mono_eval node" OFF)
if(BUILD_RVIO2_MONO_EVAL)
  add_executable(rvio2_mono_eval src/rvio2_mono_eval.cc)
  ament_target_dependencies(rvio2_mono_eval
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    visualization_msgs
    tf2_ros
    rosbag2_cpp
    rosbag2_storage
    rosbag2_transport
    Eigen3
    OpenCV
  )
  target_link_libraries(rvio2_mono_eval ${PROJECT_NAME})
endif()

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

install(TARGETS
  rvio2_mono
  # rvio2_mono_eval
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
