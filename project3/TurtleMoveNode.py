import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from math import atan2, pi
import time

class MoveToPersonNode(Node):
    def __init__(self):
        super().__init__('move_to_person_node')

        # Subscription to /person_location 
        self.subscription = self.create_subscription(
            Point,  # Assuming the message type is geometry_msgs/Point
            '/person_location',
            self.person_location_callback,
            10  # Queue size
        )

        # Publisher to control the TurtleBot3's movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("MoveToPersonNode has started and is listening to /person_location")

    def person_location_callback(self, msg):
        """
        Callback function for processing the received person's location
        and commanding the TurtleBot3 to move to that location.
        """
        person_x = msg.x
        person_y = msg.y
        self.get_logger().info(f"Received person location: ({person_x}, {person_y})")

        # Get the current position of the robot (For simplicity, assuming the robot starts at (0, 0))
        # In a real scenario, you would use odometry data
        robot_x, robot_y = 0, 0  # Example values; update with actual robot's position

        # Calculate the direction to turn
        angle_to_target = atan2(person_y - robot_y, person_x - robot_x)
        current_angle = 0  # Assuming the robot starts facing the positive X axis

        # Calculate the angular difference
        angle_diff = angle_to_target - current_angle
        if angle_diff > pi:
            angle_diff -= 2 * pi
        elif angle_diff < -pi:
            angle_diff += 2 * pi

        self.rotate_robot(angle_diff)

        # Move towards the target location after turning
        self.move_robot(person_x, person_y)

    def rotate_robot(self, angle_diff):
        """
        Rotate the robot by the specified angle.
        Positive angle_diff means turn counterclockwise, negative means clockwise.
        """
        twist = Twist()
        twist.angular.z = 0.5 if angle_diff > 0 else -0.5  # Rotate at a constant speed
        self.cmd_vel_pub.publish(twist)

        # Allow the robot to rotate until the angle difference is small enough
        time.sleep(abs(angle_diff) / 0.5)  # Adjust rotation time based on desired speed
        twist.angular.z = 0  # Stop rotating
        self.cmd_vel_pub.publish(twist)

    def move_robot(self, target_x, target_y):
        """
        Move the robot forward towards the target position.
        """
        twist = Twist()
        twist.linear.x = 0.2  # Move at a constant speed

        # In this example, we simply move the robot straight towards the target
        # In a real scenario, you should include logic for stopping at the target
        distance_to_target = ((target_x ** 2) + (target_y ** 2)) ** 0.5
        while distance_to_target > 0.1:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # Adjust to ensure the robot doesn't move too fast

        # Stop the robot once it reaches the target
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Arrived at the target location")

def main(args=None):
    rclpy.init(args=args)
    node = MoveToPersonNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
