#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_service/srv/least_squares.hpp"

#include <memory>

void add(const std::shared_ptr<linear_algebra_service::srv::LeastSquares::Request> request,
          std::shared_ptr<linear_algebra_service::srv::LeastSquares::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  // Initializes ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // Create a node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("least_squares_server");

  // Creates a service named least_squares for that node and automatically advertises it over the networks with the &add method:
  rclcpp::Service<linear_algebra_service::srv::LeastSquares>::SharedPtr service =
    node->create_service<linear_algebra_service::srv::LeastSquares>("least_squares", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to calculate least squares.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}