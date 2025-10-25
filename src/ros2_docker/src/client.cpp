#include "rclcpp/rclcpp.hpp"
#include "ros2_docker/srv/sum_ints.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create node
  auto node = rclcpp::Node::make_shared("sum_client");

  // Declare expected parameters
  int64_t a = node->declare_parameter<int64_t>("a", 0);
  int64_t b = node->declare_parameter<int64_t>("b", 0);

  a = node->get_parameter("a").as_int();
  b = node->get_parameter("b").as_int();

  RCLCPP_INFO(node->get_logger(), "Using a = %ld, b = %ld", a, b);

  auto client = node->create_client<ros2_docker::srv::SumInts>("sum_ints");

  auto request = std::make_shared<ros2_docker::srv::SumInts::Request>();
  request->a = a;
  request->b = b;

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node->get_logger(), "Waiting for service...");
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
