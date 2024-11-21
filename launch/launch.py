from launch import LaunchDescription  # Importing LaunchDescription class
from launch_ros.actions import Node  # Importing Node action

def generate_launch_description():
    # Define a launch description generator function

    # Node for object detection, subscribing to TurtleBot3's /scan topic
    object_detection_node = Node(
        package='project3',
        executable='ObjectDetectionNode'  # Remap /scan topic if necessary, this was node1
    )

    ld = LaunchDescription([
        object_detection_node
    ])

    return ld  # Return the generated launch description
