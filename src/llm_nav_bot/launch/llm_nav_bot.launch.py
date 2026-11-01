"""行動層・安全層・LLM 指令・ロガーをまとめて起動する。

Gazebo + ros_gz_bridge は別途起動しておく (Nav2 は起動しない)。
  ros2 launch llm_nav_bot llm_nav_bot.launch.py                 # Claude
  ros2 launch llm_nav_bot llm_nav_bot.launch.py backend:=rule   # API なし
指示の入力は別ターミナルで: ros2 run llm_nav_bot console
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params_file = LaunchConfiguration("params_file")
    common = [params_file, {"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("llm_nav_bot"), "config", "params.yaml"])),
        DeclareLaunchArgument("backend", default_value="claude",
                              description="claude または rule"),
        DeclareLaunchArgument("model", default_value="claude-opus-5-5"),
        DeclareLaunchArgument("logging", default_value="true"),

        Node(package="llm_nav_bot", executable="behavior_controller",
             name="behavior_controller", output="screen", parameters=common),
        Node(package="llm_nav_bot", executable="safety_layer",
             name="safety_layer", output="screen", parameters=common),
        Node(package="llm_nav_bot", executable="llm_commander",
             name="llm_commander", output="screen",
             parameters=common + [{"backend": LaunchConfiguration("backend"),
                                   "model": LaunchConfiguration("model")}]),
        Node(package="llm_nav_bot", executable="data_logger",
             name="data_logger", output="screen", parameters=common,
             condition=IfCondition(LaunchConfiguration("logging"))),
    ])
