cmake_minimum_required(VERSION 3.5)
project(linear_algebra_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LeastSquares.srv"
)

add_executable(server src/least_squares_server.cpp)
ament_target_dependencies(server rclcpp)

add_executable(client src/least_squares_client.cpp)
ament_target_dependencies(client rclcpp)

# Link the generated interfaces
rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(client ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    server
    client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
