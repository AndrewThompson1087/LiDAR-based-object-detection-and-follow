from launch import LaunchDescription  # Importing LaunchDescription class
from launch_ros.actions import Node  # Importing Node action

def generate_launch_description():
    # Define a launch description generator function

    # Node for object detection, subscribing to TurtleBot3's /scan topic
    object_detection_node = Node(
        package='project3',
        executable='ObjectDetectionNode',
        remappings=[('/scan', '/scan')]  # Remap /scan topic if necessary, this was node1
    )

    # Node for object tracking
    object_tracking_node = Node(
        package='project3',
        executable='ObjectTrackingNode'
    ) # This is node2
    
    arg = DeclareLaunchArgument('arg_name')  
    # Declare a launch argument named 'arg_name'

    ld = LaunchDescription([
        arg,  # Add the launch argument to the launch description
        object_detection_node,
        object_tracking_node
    ])

    return ld  # Return the generated launch description
