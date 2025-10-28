cmake_minimum_required(VERSION 3.5)
project(ur20_display)

# Use C++14 or later
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rviz_visual_tools REQUIRED)

# Executable
add_executable(state_publisher src/state_publisher.cpp)
ament_target_dependencies(state_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  rviz_visual_tools
)
target_include_directories(state_publisher PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Install the executable (this is crucial)
install(TARGETS
  state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, config, and rviz directories
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
