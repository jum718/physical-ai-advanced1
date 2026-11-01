"""TurtleBot3 (Burger) を Gazebo で起動する (Nav2 なし)。

公式の turtlebot3_world.launch.py は GUI が必須で、画面のない環境では GUI の
クラッシュで全体が落ちる。これはサーバのみ (ヘッドレス) でも起動できるようにしたもの。

  ros2 launch llm_nav_bot sim.launch.py              # ヘッドレス
  ros2 launch llm_nav_bot sim.launch.py gui:=true    # GUI あり (WSLg が必要。動画撮影用)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb3 = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    launch_dir = os.path.join(tb3, "launch")
    world = LaunchConfiguration("world")
    gz_sim = PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py"))

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        DeclareLaunchArgument(
            "world", default_value=os.path.join(tb3, "worlds", "turtlebot3_world.world")),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(tb3, "models")),

        IncludeLaunchDescription(gz_sim, launch_arguments={
            "gz_args": ["-r -s -v2 ", world], "on_exit_shutdown": "true"}.items()),
        IncludeLaunchDescription(
            # GUI を閉じてもシミュレーション本体は止めない (再表示は `gz sim -g`)
            gz_sim, launch_arguments={"gz_args": "-g -v2 ", "on_exit_shutdown": "false"}.items(),
            condition=IfCondition(LaunchConfiguration("gui"))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "spawn_turtlebot3.launch.py")),
            launch_arguments={"x_pose": LaunchConfiguration("x_pose"),
                              "y_pose": LaunchConfiguration("y_pose")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "robot_state_publisher.launch.py")),
            launch_arguments={"use_sim_time": "true"}.items()),
    ])
