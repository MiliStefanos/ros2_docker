#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_msgs/srv/least_squares.hpp"
#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int ParseYamlFile()
{
  YAML::Node config = YAML::LoadFile("/ros2_ws/src/linear_algebra_service/config/input.yaml");
  if (!config["rows_num"]) {
    throw std::runtime_error("Missing keys in input.yaml");
  }

  auto rows_num = config["rows_num"].as<int>();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Client read from yaml file the number of matrix rows: %d", rows_num);
  return rows_num;
}

std::pair<Eigen::MatrixXf, Eigen::VectorXf> CreateRandomMatrixAndVector(int rows_num)
{
  // Eigen::MatrixXf matrix = Eigen::MatrixXf::Random(rows_num, 3);
  // std::cout << "Here is the matrix:\n" << matrix << std::endl;
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Random generated matrix: %s", matrix.str().c_str());
  // Eigen::VectorXf vector = Eigen::VectorXf::Random(rows_num);
  // std::cout << "Here is the vector:\n" << vector << std::endl;
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Random generated vector: %s", vector.str().c_str());

  Eigen::MatrixXf matrix = Eigen::MatrixXf::Random(rows_num, 3);
  Eigen::VectorXf vector = Eigen::VectorXf::Random(rows_num);

  std::stringstream ss_matrix;
  ss_matrix << matrix;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Random generated matrix:\n%s", ss_matrix.str().c_str());

  std::stringstream ss_vector;
  ss_vector << vector;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Random generated vector:\n%s", ss_vector.str().c_str());

  return {matrix, vector};
}

std::vector<float> FlattenMatrix(const Eigen::MatrixXf& matrix) {
  std::vector<float> flattened;
  flattened.reserve(matrix.rows() * matrix.cols());

  for (auto col = 0; col < matrix.cols(); ++col) {
    for (auto row = 0; row < matrix.rows(); ++row) {
      flattened.push_back(matrix(row, col));
    }
  }
  return flattened;
}

std::vector<float> FlattenVector(const Eigen::VectorXf& vector) {
  std::vector<float> flattened;
  flattened.reserve(vector.size());

  for (auto i = 0; i < vector.size(); ++i) {
    flattened.push_back(vector(i));
  }
  return flattened;
}
 
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "stleast_squares_client started");

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("least_squares_client");
  rclcpp::Client<linear_algebra_msgs::srv::LeastSquares>::SharedPtr client =
    node->create_client<linear_algebra_msgs::srv::LeastSquares>("least_squares");

  auto publisher = node->create_publisher<geometry_msgs::msg::Vector3>("least_squares_solution", 10);

  // Load params from yaml file
  auto rows_num = ParseYamlFile();

  if (rows_num < 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Rows of matrix less than 3. rows Num: %d", rows_num);
    return 0;
  }

  auto [matrix, vector] = CreateRandomMatrixAndVector(rows_num);

  // Call server 
  auto request = std::make_shared<linear_algebra_msgs::srv::LeastSquares::Request>();
  request->matrix_vectorized = FlattenMatrix(matrix);
  request->rows_num = rows_num;
  request->vector = FlattenVector(vector);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto response = client->async_send_request(request);

  //Wait for the result.
  if (rclcpp::spin_until_future_complete(node, response) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Client received response from server");

    auto result = response.get();
    geometry_msgs::msg::Vector3 x_prime_vec = result->x_prime_rotated_displaced;
    geometry_msgs::msg::Quaternion rotation  = result->rotation;
    geometry_msgs::msg::Vector3 displacement  = result->displacement;

    Eigen::Vector3f x_prime(x_prime_vec.x, x_prime_vec.y, x_prime_vec.z);
    Eigen::Vector3f d_prime(displacement.x, displacement.y, displacement.z);

    Eigen::Quaternionf R_prime(rotation.w, rotation.x, rotation.y, rotation.z);
    Eigen::Matrix3f R = R_prime.toRotationMatrix();

    // Inverse transform
    Eigen::Vector3f x = R.inverse() * (x_prime - d_prime);

    std::stringstream ss_x;
    ss_x << x;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The least-squares solution is:\n%s", ss_x.str().c_str());

    std::stringstream ss_R_prime;
    ss_R_prime << R_prime;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The R' matrix is:\n%s", ss_R_prime.str().c_str());

    std::stringstream ss_d_prime;
    ss_d_prime << d_prime;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The d' vector:\n%s", ss_d_prime.str().c_str());

    geometry_msgs::msg::Vector3 x_msg;
    x_msg.x = x.x();
    x_msg.y = x.y();
    
    x_msg.z = x.z();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing x to topic...");
    for (int i = 0; i < 10; ++i) {
      publisher->publish(x_msg);
      rclcpp::sleep_for(500ms);
    }

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
 
  rclcpp::shutdown();
  return 0;
}