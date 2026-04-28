from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tour_manager',
            namespace='',
            executable='tour_manager',
            name='tour_manager'
        ),
        Node(
            package='tour_manager',
            namespace='',
            executable='tour_saver',
            name='tour_saver'
        ),
        Node(
            package='robot_tour',
            namespace='robot_tour',
            executable='tour_guide_start',
        )
    ])
