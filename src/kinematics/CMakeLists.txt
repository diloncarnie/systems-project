cmake_minimum_required(VERSION 3.8)
project(kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# Include directories for headers
include_directories(
  ${PROJECT_SOURCE_DIR}
)

# Build library
add_library(${PROJECT_NAME} SHARED
  src/kinematics/KinematicsClass.cpp
)

add_executable(kinematics_node src/kinematics_node.cpp)
ament_target_dependencies(kinematics_node rclcpp std_msgs geometry_msgs)
target_link_libraries(kinematics_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

install(TARGETS
  kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
