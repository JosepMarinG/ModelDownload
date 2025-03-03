import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import time
import re


def parse_command(command):
    """Parses a move command from a string."""
    match = re.match(r"move\{x=(-?\d+\.\d+),y=(-?\d+\.\d+),z=(-?\d+\.\d+)\}", command)
    if match:
        x, y, z = map(float, match.groups())
        return x, y, z
    return None


def parse_wait(command):
    """Parses a wait command from a string."""
    match = re.match(r"wait\((\d+\.\d+)\)", command)
    if match:
        return float(match.group(1))
    return None


def parse_talk(command):
    """Parses a talk command from a string."""
    match = re.match(r'talk\("(.+)"\)', command)
    if match:
        return match.group(1)
    return None


def parse_get_image(command):
    """Detects if the command is getImage()."""
    return command.strip() == "getImage()"


class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.talk_publisher = self.create_publisher(String, 'robot_speech', 10)
        self.image_request_publisher = self.create_publisher(CompressedImage, 'image_for_model', 10)

        # Subscribers
        self.model_output_subscription = self.create_subscription(
            String, 'modelOutput', self.listener_callback, 10)

        self.image_subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10)

        self.commands = []
        self.latest_image = None  # Store the latest received image

    def listener_callback(self, msg):
        """Callback to receive commands from modelOutput topic."""
        self.commands = msg.data.split('\n')
        self.run_commands()

    def image_callback(self, msg):
        """Callback to store the latest image received from the camera."""
        self.latest_image = msg

    def run_commands(self):
        """Executes the movement, wait, talk, and getImage commands in sequence."""
        for command in self.commands:
            # Handle move command
            move_values = parse_command(command)
            if move_values is not None:
                x, y, z = move_values
                twist_msg = Twist()
                twist_msg.linear.x = x
                twist_msg.linear.y = y
                twist_msg.angular.z = z
                self.cmd_vel_publisher.publish(twist_msg)
                self.get_logger().info(f"Moving: x={x}, y={y}, z={z}")

            # Handle wait command
            wait_time = parse_wait(command)
            if wait_time is not None:
                self.get_logger().info(f"Waiting for {wait_time} seconds")
                time.sleep(wait_time)

            # Handle talk command
            talk_text = parse_talk(command)
            if talk_text is not None:
                speech_msg = String()
                speech_msg.data = talk_text
                self.talk_publisher.publish(speech_msg)
                self.get_logger().info(f"Speaking: {talk_text}")

            # Handle getImage command
            if parse_get_image(command):
                if self.latest_image is not None:
                    self.image_request_publisher.publish(self.latest_image)
                    self.get_logger().info("Sent latest image to the model.")
                else:
                    self.get_logger().info("No image available yet.")

        # Stop the robot after finishing the sequence
        self.cmd_vel_publisher.publish(Twist())
        self.get_logger().info("Stopping movement.")


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
