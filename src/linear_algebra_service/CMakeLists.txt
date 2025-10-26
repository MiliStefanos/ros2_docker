cmake_minimum_required(VERSION 3.5)
project(linear_algebra_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp REQUIRED)
find_package(linear_algebra_msgs REQUIRED)

# Generate service
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/LeastSquares.srv"
#   DEPENDENCIES geometry_msgs
# )

add_executable(server src/least_squares_server.cpp)
ament_target_dependencies(server rclcpp geometry_msgs Eigen3 linear_algebra_msgs)

add_executable(client src/least_squares_client.cpp)
ament_target_dependencies(client rclcpp yaml-cpp geometry_msgs Eigen3 linear_algebra_msgs)
target_link_libraries(client yaml-cpp Eigen3::Eigen)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# Link the generated interfaces
# rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")
# rosidl_target_interfaces(client ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
