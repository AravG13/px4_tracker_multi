cmake_minimum_required(VERSION 3.8)
project(drone_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# C++ executables - FIXED: Added tracker_main.cpp with main() function
add_executable(tracker_node 
  src/tracker_main.cpp
  src/tracker.cpp 
  src/object_tracker.cpp
)
ament_target_dependencies(tracker_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  cv_bridge 
  image_transport
  OpenCV
)

add_executable(gstreamer_camera_node 
  src/gstreamer_camera_node.cpp
)
ament_target_dependencies(gstreamer_camera_node 
  rclcpp 
  sensor_msgs 
  cv_bridge
  OpenCV
)

# Install C++ executables
install(TARGETS
  tracker_node
  gstreamer_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/working_px4_control.py
  scripts/safety_monitor.py
  scripts/manual_drone_control.py
  scripts/csv_logger.py
  scripts/data_logger.py
  src/multi_drone_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()