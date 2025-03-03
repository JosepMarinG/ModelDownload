import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
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


class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            String,
            'modelOutput',
            self.listener_callback,
            10)
        self.commands = []

    def listener_callback(self, msg):
        """Callback to receive commands from respuesta_api topic."""
        self.commands = msg.data.split('\n')
        self.run_commands()

    def run_commands(self):
        """Executes the movement and wait commands in sequence."""
        for command in self.commands:
            move_values = parse_command(command)
            if move_values is not None:
                x, y, z = move_values
                twist_msg = Twist()
                twist_msg.linear.x = x
                twist_msg.linear.y = y
                twist_msg.angular.z = z
                self.publisher_.publish(twist_msg)
                self.get_logger().info(f"Moving: x={x}, y={y}, z={z}")

            wait_time = parse_wait(command)
            if wait_time is not None:
                self.get_logger().info(f"Waiting for {wait_time} seconds")
                time.sleep(wait_time)
                #rclpy.sleep(wait_time)

        # Stop the robot after finishing the sequence
        self.publisher_.publish(Twist())
        self.get_logger().info("Stopping movement.")


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"move{x=1.0,y=0.0,z=0.0} \nwait(1.0)\nmove{x=0.0,y=1.0,z=0.0}\nwait(1.0)\nmove{x=-1.0,y=0.0,z=0.0}\nwait(1.0)\nmove{x=0.0,y=-1.0,z=0.0}\nwait(1.0)"