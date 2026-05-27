"""
rviz.launch.py
==============
Launches RViz2 with a pre-configured display for the line-following robot.
Run this in a second terminal AFTER simulation.launch.py is already running.

Usage:
  ros2 launch line_follower_robot rviz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("line_follower_robot")
    rviz_config = os.path.join(pkg_share, "config", "line_follower.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([rviz_node])
