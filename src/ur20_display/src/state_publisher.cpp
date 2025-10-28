#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

using namespace std::chrono_literals;

class StatePublisherNode : public rclcpp::Node
{
    public:
    StatePublisherNode()
    : Node("state_publisher_node")
    , world_str("world")
    , elbow_link_str("elbow_link")
    , gripper_str("gripper_link") 
    {   
        // Set parameters - joint configuration
        this->declare_parameter<std::vector<std::string>>("joint_configuration.name", {world_str, elbow_link_str, gripper_str});
        this->declare_parameter<std::vector<double>>("joint_configuration.position", {0, 0, 0});

        // Get parameters
        joint_names_ = this->get_parameter("joint_configuration.name").as_string_array();
        joint_positions_ = this->get_parameter("joint_configuration.position").as_double_array();
        RCLCPP_INFO(this->get_logger(), "params joint_names: %s", joint_names_.at(0).c_str());
        RCLCPP_INFO(this->get_logger(), "params joint_positions_: %.2f", joint_positions_.at(0));


        // Publisher
        joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer
        timer_ = this->create_wall_timer(500ms, std::bind(&StatePublisherNode::time_callback, this));

    }

    ~StatePublisherNode()
    {}

    void time_callback()
    {
        // Publish joint configuration
        sensor_msgs::msg::JointState jointConfiguration;
        jointConfiguration.header.stamp = this->now();
        jointConfiguration.name = joint_names_;
        jointConfiguration.position = joint_positions_;
        joint_publisher_->publish(jointConfiguration);

        try {
            //lookupTransform(toFrameRel, fromFrameRel,tf2::TimePointZero);
            auto Tf_elbow_gripper = tf_buffer_->lookupTransform(elbow_link_str, gripper_str, tf2::TimePointZero);
            auto Tf_world_elbow = tf_buffer_->lookupTransform(world_str, elbow_link_str, tf2::TimePointZero);
            auto Tf_world_gripper = tf_buffer_->lookupTransform(world_str, gripper_str, tf2::TimePointZero);

            // convert to “Eigen::Isometry3d”
            Eigen::Isometry3d Tf_elbow_gripper_eigen = tf2::transformToEigen(Tf_elbow_gripper);
            Eigen::Isometry3d Tf_world_elbow_eigen = tf2::transformToEigen(Tf_world_elbow);
            Eigen::Isometry3d Tf_world_gripper_eigen = tf2::transformToEigen(Tf_world_gripper);

            // verify that Tf_world_gripper is correct
            Eigen::Isometry3d Tf_world_gripper_expected = Tf_world_elbow_eigen * Tf_elbow_gripper_eigen;
            bool isCorrectTransformation = Tf_world_gripper_expected.isApprox(Tf_world_gripper_eigen, 1e-3);
            RCLCPP_INFO(this->get_logger(), "Tf world-gripper is : %s", isCorrectTransformation ? "Correct" : "Fault");

            // Publish Tf_world_gripper as a x-y-z oriented frame and also 
            visual_tools_->publishAxisLabeled(Tf_world_gripper_eigen, "Tf_world_gripper");
            visual_tools_->publishText(Tf_elbow_gripper_eigen, "Tf_elbow_gripper");
            visual_tools_->trigger();


        } catch (const tf2::TransformException & ex) 
        {
          RCLCPP_INFO(this->get_logger(), "Error with transformations: %s", ex.what());
          return;
        }
    }

    std::string joint_configuration_;
    std::string world_str, elbow_link_str, gripper_str;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::shared_ptr<rviz_visual_tools::RvizVisualTools> visual_tools_;

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

//node that
// read from the parameter server a joint configuration - OK
// publish that joint configuration as a sensor_msgs/msg/JointState. - OK
// create tf listener: reads the 
//  - the transform of the gripper link relative to the robot’s elbow link (Tf_elbow_gripper), 
//  - the transform of the elbow link relative to the world frame (Tf_world_elbow) and 
//  - the transform of the gripper relative to the world frame (Tf_world_gripper)
// converte these tranforms to “Eigen::Isometry3d” and
// use Tf_elbow_gripper and Tf_world_elbow to verify that Tf_world_gripper is correct.

// Then use the rviz_visual_tools package to 
// - publish Tf_world_gripper as a x-y-z oriented frame and also 
// - publish a text label with the name “Tf_elbow_gripper”.
int main(int argc, char **argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<StatePublisherNode>());
rclcpp::shutdown();
return 0;
}