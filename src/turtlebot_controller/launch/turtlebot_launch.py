import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # รัน lidar_node จากแพ็กเกจ lidar_tracker
        Node(
            package='lidar_tracker',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),
        
        # รัน move_node จากแพ็กเกจ turtlebot_controller
        Node(
            package='turtlebot_controller',
            executable='move_node',
            name='move_node',
            output='screen'
        ),
    ])
