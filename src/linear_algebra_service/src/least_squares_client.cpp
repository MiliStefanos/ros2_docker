#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_service/srv/least_squares.hpp"

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

struct InputData
{
  std::vector<geometry_msgs::msg::Vector3> m_matrix{};
  geometry_msgs::msg::Vector3 vector;
};

InputData ParseYamlFile()
{
  YAML::Node config = YAML::LoadFile("/ros2_ws/src/linear_algebra_service/config/input.yaml");
  if (!config["matrix"] || !config["vector"]) {
    throw std::runtime_error("Missing keys in input.yaml");
  }

  std::vector<geometry_msgs::msg::Vector3> matrix;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mx3 matrix: ");
  for (const auto& row : config["matrix"]) {
    geometry_msgs::msg::Vector3 v;
    v.x = row[0].as<double>();
    v.y = row[1].as<double>();
    v.z = row[2].as<double>();
    matrix.push_back(v);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%.1f, %.1f, %.1f]", v.x, v.y, v.z);
  }

  auto vec = config["vector"];
  geometry_msgs::msg::Vector3 v;
  v.x = vec[0].as<double>();
  v.y = vec[1].as<double>();
  v.z = vec[2].as<double>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vector: [%.1f, %.1f, %.1f]", v.x, v.y, v.z);
  return InputData{matrix, v};
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "stleast_squares_client started");

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("least_squares_client");
  rclcpp::Client<linear_algebra_service::srv::LeastSquares>::SharedPtr client =
    node->create_client<linear_algebra_service::srv::LeastSquares>("least_squares");

  // Load params from yaml file
  auto inputData = ParseYamlFile();

  // Call server with input Data
  auto request = std::make_shared<linear_algebra_service::srv::LeastSquares::Request>();
  request->matrix = inputData.m_matrix;
  request->vector = inputData.vector;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  //Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Client received response from server");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  //auto result = client->async_send_request(request);
  // Wait for the result.
  // if (rclcpp::spin_until_future_complete(node, result) ==
  //   rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service least_squares");
  // }

  rclcpp::shutdown();
  return 0;
}