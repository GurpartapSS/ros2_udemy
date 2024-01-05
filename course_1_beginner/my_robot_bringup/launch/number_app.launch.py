## min code template
# from launch import LaunchDescription
# def generate_launch_description():
#     ld = LaunchDescription()
#     return ld

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remap_count = ("number_count", "my_num_count")

    number_pub_node = Node(
        package="my_pkg_cpp",
        executable="number_publisher_param",
        name="my_num_pb",
        remappings = [
            ("number", "my_number")
            ],
        parameters = [
            {"number_to_publish": 5},
            {"publish_frequency": 2.0}
        ]
    )

    number_counter_node = Node(
        package="activity_pkgs",
        executable="a2_numberCounter",
        name="my_num_counter",
        remappings = [
            ("number", "my_number"),
            remap_count
        ]
    )

    ld.add_action(number_pub_node)
    ld.add_action(number_counter_node)
    return ld

