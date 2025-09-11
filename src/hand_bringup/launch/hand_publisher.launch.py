from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Control node and publisher node
        Node(
            package='inspire_hand_ros',
            executable='finger_control_node',
            name='finger_control_node'

        ),

        # headless driver node
        Node(
            package='inspire_hand_ros',
            executable='headless_driver_node',
            name='headless_driver_node'

        ),

        # visualizer node
        Node(
            package='inspire_hand_ros',
            executable='inspire_hand_visualizer_node',
            name='inspire_hand_visualizer_node'
        ),

        
    ])

'''
'console_scripts': [
            'dds_publisher_node = inspire_hand_ros.dds_publisher_node:main',
            'inspire_hand_visualizer_node = inspire_hand_ros.inspire_hand_visualizer_node:main',
            'headless_driver_node = inspire_hand_ros.headless_driver_node:main',
            'finger_control_node = inspire_hand_ros.finger_control_node:main'
            
        ],
    },
'''
