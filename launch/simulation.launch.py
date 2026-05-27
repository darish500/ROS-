"""
simulation.launch.py — clean version, spawn after Gazebo ready
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("line_follower_robot")
    urdf_xacro_path = os.path.join(pkg_share, "urdf", "line_follower_robot.urdf.xacro")
    world_path      = os.path.join(pkg_share, "worlds", "line_track.sdf")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_headless     = DeclareLaunchArgument("headless",     default_value="false")

    use_sim_time = LaunchConfiguration("use_sim_time")
    headless     = LaunchConfiguration("headless")

    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_xacro_path]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    gz_sim_with_gui = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-r"],
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-r", "-s"],
        output="screen",
        condition=IfCondition(headless),
    )

    # Spawn at x=0, y=-0.8 (on the bottom straight of the track)
    # No yaw argument — default facing +X which is along the straight
    spawn_robot = TimerAction(
        period=6.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_robot",
            output="screen",
            arguments=[
                "-name",  "line_follower_robot",
                "-topic", "/robot_description",
                "-x",     "0.0",
                "-y",     "-0.8",
                "-z",     "0.10",
            ],
        )],
    )

    gz_bridge = TimerAction(
        period=6.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_ros_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]",
                "/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist]",
                "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry]",
                "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V]",
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model]",
                "/line_camera/image@sensor_msgs/msg/Image[gz.msgs.Image]",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        )],
    )

    line_follower = TimerAction(
        period=9.0,
        actions=[Node(
            package="line_follower_robot",
            executable="line_follower_node.py",
            name="line_follower_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"base_speed":   0.10},
                {"kp":           0.008},
                {"kd":           0.004},
                {"black_thresh": 90},
                {"debug_image":  True},
            ],
        )],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_headless,
        robot_state_publisher_node,
        gz_sim_with_gui,
        gz_sim_headless,
        spawn_robot,
        gz_bridge,
        line_follower,
    ])
