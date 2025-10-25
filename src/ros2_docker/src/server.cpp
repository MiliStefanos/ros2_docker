#include "rclcpp/rclcpp.hpp"
#include "ros2_docker/srv/sum_ints.hpp"

using namespace std::placeholders;

class SumServer : public rclcpp::Node {
public:
  SumServer() : Node("sum_server") {
    service_ = this->create_service<ros2_docker::srv::SumInts>(
      "sum_ints", std::bind(&SumServer::handle_sum, this, _1, _2));
  }

private:
  void handle_sum(
    const std::shared_ptr<ros2_docker::srv::SumInts::Request> request,
    std::shared_ptr<ros2_docker::srv::SumInts::Response> response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld, b=%ld -> sum=%ld",
                request->a, request->b, response->sum);
  }

  rclcpp::Service<ros2_docker::srv::SumInts>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SumServer>());
  rclcpp::shutdown();
  return 0;
}