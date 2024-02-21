from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import time

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='deadpool_chase_object',
            namespace='',
            executable='detectobj',
            name='detect_object'
        ),
        # Node(
        #     package='deadpool_chase_object',
        #     namespace='',
        #     executable='viewimg',
        #     name='view_image'
        # ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                package='deadpool_chase_object',
                namespace='',
                executable='getobjrng',
                name='get_object_range'
                ),
                Node(
                package='deadpool_chase_object',
                namespace='',
                executable='chaseobj',
                name='chase_obj'
                ),
            ]),
        
    ]) 