#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_service/srv/least_squares.hpp"
#include <Eigen/Dense>

#include <memory>

void calculateLeastSquares(const std::shared_ptr<linear_algebra_service::srv::LeastSquares::Request> request,
          std::shared_ptr<linear_algebra_service::srv::LeastSquares::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service received from client");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "matrix:");
  for (const auto& row : request->matrix) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%.1f, %.1f, %.1f]", row.x, row.y, row.z);
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "3d_vector: [%.1f, %.1f, %.1f]", request->vector.x, request->vector.y, request->vector.z);


  Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
  std::cout << "Here is the matrix A:\n" << A << std::endl;
  Eigen::VectorXf b = Eigen::VectorXf::Random(3);
  std::cout << "Here is the right hand side b:\n" << b << std::endl;
  std::cout << "The least-squares solution is:\n"
            << A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) << std::endl;
  // Calculate 
  // response->sum = request->a + request->b;
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
  //               request->a, request->b);
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  // Initializes ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // Create a node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("least_squares_server");

  // Creates a service named least_squares for that node and automatically advertises it over the networks with the &calculateLeastSquares method:
  rclcpp::Service<linear_algebra_service::srv::LeastSquares>::SharedPtr service =
    node->create_service<linear_algebra_service::srv::LeastSquares>("least_squares", &calculateLeastSquares);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to calculate least squares.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}