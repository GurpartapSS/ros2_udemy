from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["gugu","gavy","tony","alfred","shanti"]

    nodes = []
    for name in robot_names:
        nodes.append(Node(
            package="my_pkg_cpp",
            executable="robot_news_station",
            name = "robot_news_station_" + name,
            parameters = [
                {"robot_name": name}
            ]
        ))

    smartphone = Node(
        package="my_pkg_cpp",
        executable="smartPhone",
    )

    for i in nodes:
        ld.add_action(i)
    ld.add_action(smartphone)
    return ld