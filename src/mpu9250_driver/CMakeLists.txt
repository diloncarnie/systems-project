cmake_minimum_required(VERSION 3.8)
project(mpu9250_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories for headers
include_directories(
  ${PROJECT_SOURCE_DIR}
)

# Build library
add_library(${PROJECT_NAME} SHARED
  src/mpu9250_driver/Mpu9250Class.cpp
)

# # Build library
# add_library(mpu9250_component SHARED
#   src/mpu9250_node.cpp
# )
# ament_target_dependencies(my_component
#   rclcpp
#   rclcpp_components
#   sensor_msgs
# )

# # Also add a node executable which simply loads the component
# rclcpp_components_register_node(my_component
#   PLUGIN "mpu9250_driver::Mpu9250Node"
#   EXECUTABLE my_node
# )

add_executable(mpu9250_node src/mpu9250_node.cpp)
ament_target_dependencies(mpu9250_node rclcpp sensor_msgs)
target_link_libraries(mpu9250_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

install(TARGETS
  mpu9250_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
