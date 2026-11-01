from glob import glob

from setuptools import find_packages, setup

package_name = "llm_nav_bot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PAI student",
    maintainer_email="student@example.com",
    description="Natural-language navigation for TurtleBot3 with an LLM planner and a LiDAR safety layer",
    license="MIT",
    entry_points={
        "console_scripts": [
            "behavior_controller = llm_nav_bot.behavior_controller_node:main",
            "safety_layer = llm_nav_bot.safety_layer_node:main",
            "llm_commander = llm_nav_bot.llm_commander_node:main",
            "data_logger = llm_nav_bot.data_logger_node:main",
            "console = llm_nav_bot.command_console:main",
            "plot_run = llm_nav_bot.tools.plot_run:main",
            "try_planner = llm_nav_bot.tools.try_planner:main",
        ],
    },
)
