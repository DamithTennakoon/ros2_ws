#/!usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class KeyboardInput(Node):

    def __init__(self):
        super().__init__("keyboard_input")

        # Construct a publish node
        self._publish_key_input = self.create_publisher(Float64, 'input_data', 10)

        # Construct a topic for event-driven mechanism
        self._subscription = self.create_subscription(Float64, 'my_trigger_topic', self.callback, 10, self._subscription)

    # Define callback method
        def callback(self, msg):
            pass
        


def main(args=None):
    rclpy.init(args=args)

    node = KeyboardInput()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()