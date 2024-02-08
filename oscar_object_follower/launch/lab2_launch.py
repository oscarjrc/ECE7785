from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oscar_object_follower',
            namespace='',
            executable='findobj',
            name='find_object'
        ),
        Node(
            package='oscar_object_follower',
            namespace='',
            executable='rotrob',
            name='rotate_robot'
        ),    
    ]) 
