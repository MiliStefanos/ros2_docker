cmake_minimum_required(VERSION 3.5)
project(ros2_docker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SumInts.srv"
)

# Add executables
add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp)

# Link generated interfaces
rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(client ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()