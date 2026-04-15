from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    # --- OpenNI2 Camera Launch ---
    openni_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                '/opt/ros/humble/share/openni2_camera/launch',
                'camera_only.launch.py'
            )
        )
    )

    camera_bridge = Node(
        package='my_robot',
        executable='camera_node',
        name='camera_bridge',
        output='screen'
    )

    uart_bridge = Node(
    package='my_robot',
    executable='uart_node',
    name='uart_bridge',
    output='log'
    )

    return LaunchDescription([
        openni_launch,
        camera_bridge,
        uart_bridge
    ])
