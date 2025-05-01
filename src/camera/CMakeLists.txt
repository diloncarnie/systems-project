cmake_minimum_required(VERSION 3.5)
project(camera)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(camera src/camera.cpp)
target_include_directories(camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(camera rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs visualization_msgs OpenCV)
target_link_libraries(camera ${OpenCV_LIBS})

# Install
install(TARGETS
  camera
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
script/serial_bridge.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
 #ament_lint_auto_find_test_dependencies()
endif()

ament_package()