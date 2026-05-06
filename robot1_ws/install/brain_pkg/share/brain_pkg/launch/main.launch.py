import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'brain_pkg'

    bridge_node = Node(
        package=package_name,
        executable='bridge',
        name='bridge_node',
        output='screen' 
    )

    vision_node = Node(
        package=package_name,
        executable='vision',
        name='vision_node',
        output='screen'
    )

    navigation_node = Node(
        package=package_name,
        executable='navigation',
        name='navigation_node',
        output='screen'
    )
    
    trajectory_node = Node(
        package=package_name,
        executable='trajectory',
        name='trajectory_node',
        output='screen'
    )

    return LaunchDescription([
        bridge_node,
        vision_node,
        navigation_node,
        trajectory_node
    ])
