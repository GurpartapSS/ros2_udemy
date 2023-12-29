from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtle_start = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    turtle_spawns = Node(
        package="activity_pkgs",
        executable="project_spawnTurtle"
    )

    ld.add_action(turtle_start)
    ld.add_action(turtle_spawns)
    return ld
