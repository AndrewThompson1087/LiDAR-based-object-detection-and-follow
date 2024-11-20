from launch import LaunchDescription  # Importing LaunchDescription class
from launch_ros.actions import Node  # Importing Node action

def generate_launch_description():
    # Define a launch description generator function

    detection_and_tracking_node = Node(
        package='project3',
        executable='DetectionAndTracking',
        remappings=[('/scan', '/scan')]  # Remap /scan topic if necessary, was node1
    )
    
    arg = DeclareLaunchArgument('arg_name')  
    # Declare a launch argument named 'arg_name'

    ld = LaunchDescription([
        arg,  # Add the launch argument to the launch description
        detection_and_tracking_node  # Add the Node action to the launch description
    ])

    return ld  # Return the generated launch description
