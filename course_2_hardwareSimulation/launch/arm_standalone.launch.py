from launch import LaunchDescription, launch_description_sources, actions
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python import get_package_share_path
import os

def generate_launch_description():
    ld = LaunchDescription()

    #get path to the robot URDF file
    urdf_path = os.path.join(get_package_share_path('my_robot_description'),'urdf','standalone_arm.xacro')
    rviz_path = os.path.join(get_package_share_path('my_robot_description'),'rviz','arm_standalone.rviz')
    # robot_desc = ParameterValue(Command(['xacro ', urdf_path]),value_type=str)
    # robot_desc = Command(['xacro ', urdf_path])


    robot_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description":Command(['xacro ', urdf_path])}
            ]
    )

    # joint_states_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui"
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_path]
    )

    launch_gazeo = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join('/opt/ros/humble/share/gazebo_ros/launch/', 'gazebo.launch.py')))
            # launch_arguments={'world': world_path}.items())
    
    spawmer_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', '/robot_description','-entity', 'Maya']
    )

    ld.add_action(robot_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(launch_gazeo)
    ld.add_action(spawmer_node)
    return ld
