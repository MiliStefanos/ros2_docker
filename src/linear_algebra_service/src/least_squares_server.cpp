#include "rclcpp/rclcpp.hpp"
#include "linear_algebra_msgs/srv/least_squares.hpp"
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <memory>

using namespace std::placeholders;

class LeastSquaresNode : public rclcpp::Node
{
  public:
    LeastSquaresNode()
    : Node("least_squares_subscriber")
    {
      // Create service
      service_ = this->create_service<linear_algebra_msgs::srv::LeastSquares>("least_squares", 
        std::bind(&LeastSquaresNode::calculateLeastSquares, this, _1, _2));

      // Create subscription
      subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "least_squares_solution", 10, std::bind(&LeastSquaresNode::sub_function, this, _1));

      // Create thread
      log_thread_ = std::thread(&LeastSquaresNode::wait_thread, this);

      RCLCPP_INFO(this->get_logger(), "LeastSquaresNode initialized successfully.");
    }

    ~LeastSquaresNode()
    {
      shutdown_ = true;
      condition_variable_.notify_all();
      if (log_thread_.joinable())
      {
        log_thread_.join();
      }
    }

  private:
    
    rclcpp::Service<linear_algebra_msgs::srv::LeastSquares>::SharedPtr service_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;

    std::mutex mutex_;
    std::condition_variable condition_variable_;
    std::thread log_thread_;
    geometry_msgs::msg::Vector3 last_msg_;
    bool notified_;
    std::atomic<bool> shutdown_{false};

    void sub_function(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
      std::unique_lock<std::mutex> lock(mutex_);
      last_msg_ = *msg;
      notified_ = true;

      // notify thread through condition variable
      condition_variable_.notify_all();
    }

    void wait_thread()
    {
      std::unique_lock<std::mutex> lock(mutex_);
      while (!shutdown_)
      {
        condition_variable_.wait(lock, [this] { return notified_ || shutdown_; });
        if (shutdown_)
          break;

        
        RCLCPP_INFO(this->get_logger(), "Server's thread received a vector: [%.6f, %.6f, %.6f]", last_msg_.x, last_msg_.y, last_msg_.z);
        notified_ = false;
      }
    }

    void calculateLeastSquares(const std::shared_ptr<linear_algebra_msgs::srv::LeastSquares::Request> request,
              std::shared_ptr<linear_algebra_msgs::srv::LeastSquares::Response> response)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service received from client");

      Eigen::Map<const Eigen::VectorXf> b(request->vector.data(), request->rows_num);
      Eigen::Map<const Eigen::MatrixXf> A(request->matrix_vectorized.data(), request->rows_num, 3);

      // std::cout << "Here is the matrix A:\n" << A << std::endl;
      // std::cout << "Here is the vector b:\n" << b << std::endl;

      if (A.rows() != b.rows())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "A matrix rows [%ld] != b vector size [%ld]", A.rows(), b.rows());
        std::cout << "Cannot calculate least squares solution" << std::endl;
        return;
      }

      Eigen::MatrixXf A_T = A.transpose();

      Eigen::Vector3f x = (A_T * A).ldlt().solve(A_T * b); // Least squares solution: x = (AᵀA)^(-1) Aᵀb
      //Eigen::Vector3f x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

      // rotation matrix R'
      Eigen::Matrix3f  R_prime = Eigen::Quaternionf::UnitRandom().toRotationMatrix();
      
      // displaycement vector d'
      Eigen::Vector3f d_prime = Eigen::Vector3f::Random();

      // x'
      Eigen::Vector3f x_prime = (R_prime * x) + d_prime;

      // std::cout << "The least-squares solution x is:\n" << x << std::endl;
      // std::cout << "The rotation matrix R' is:\n" << R_prime << std::endl;
      // std::cout << "The displaycement vector is:\n" << d_prime << std::endl;
      
      // === Convert to LeastSquares response types ===
      geometry_msgs::msg::Vector3 x_prime_msg;
      x_prime_msg.x = x_prime.x();
      x_prime_msg.y = x_prime.y();
      x_prime_msg.z = x_prime.z();

      geometry_msgs::msg::Vector3 d_prime_msg;
      d_prime_msg.x = d_prime.x();
      d_prime_msg.y = d_prime.y();
      d_prime_msg.z = d_prime.z();

      Eigen::Quaternionf q(R_prime);

      geometry_msgs::msg::Quaternion rotation_msg;
      rotation_msg.w = q.w();
      rotation_msg.x = q.x();
      rotation_msg.y = q.y();
      rotation_msg.z = q.z();

      // === Fill response ===
      response->x_prime_rotated_displaced = x_prime_msg;
      response->displacement = d_prime_msg;
      response->rotation = rotation_msg;
    }
};



int main(int argc, char **argv)
{
  // Initializes ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // Create LeastSquaresNode with subscriber and thread
  auto node = std::make_shared<LeastSquaresNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  
  rclcpp::shutdown();
}