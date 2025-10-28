from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch configuration variables
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    name = LaunchConfiguration("name")

    # Package paths
    ur_pkg_share = FindPackageShare("ur_description")
    display_pkg_share = FindPackageShare("ur20_display")

    # Arguments
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur20"),
        DeclareLaunchArgument("name", default_value="ur"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20.0"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    Command([
                        "xacro ",
                        PathJoinSubstitution([ur_pkg_share, "urdf", "ur.urdf.xacro"]),
                        " ",
                        "name:=", name,
                        " ",
                        "ur_type:=", ur_type,
                        " ",
                        "safety_limits:=", safety_limits,
                        " ",
                        "safety_pos_margin:=", safety_pos_margin,
                        " ",
                        "safety_k_position:=", safety_k_position
                    ]),
                    value_type=str
                )
            }]
        ),

        Node(
            package="ur20_display",
            executable="state_publisher",
            name="state_publisher",
            output="screen",
            parameters=[
                PathJoinSubstitution([display_pkg_share, "config", "joint_configuration.yaml"])
            ]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=[
                "-d", PathJoinSubstitution([display_pkg_share, "rviz", "ur20_view.rviz"])
            ]
        ),
    ])
