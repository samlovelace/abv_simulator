from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
        # Launch executable from the first package
        Node(
            package='abv_comms',  # Replace with the actual package name
            executable='thruster_control_ex',  # Replace with the actual executable name
            name='thruster_control',  # Node name (optional)
            output='screen',  # Output options: 'screen' or 'log'
        ),

        # Launch executable from the second package
        Node(
            package='abv_controller',  # Replace with the actual package name
            executable='abv_controller',  # Replace with the actual executable name
            name='controller',  # Node name (optional)
            output='screen',
        ),
        # Launch executable from the second package
        Node(
            package='abv_simulator',  # Replace with the actual package name
            executable='abv_simulator',  # Replace with the actual executable name
            name='simulator',  # Node name (optional)
            output='screen',
        ),
    ])
