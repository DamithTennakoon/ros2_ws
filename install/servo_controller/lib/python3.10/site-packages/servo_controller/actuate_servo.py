#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 

from std_msgs.msg import Float64 

class ActuateServo(Node):
    
    def __init__(self):
        super().__init__("actuate_servo")

        # Create a subscriber object
        self._receive_key_data = self.create_subscription(Float64, 'input_data', self.parse_keyboard_data, 10)

        # Create an object to store keyboard data
        self._key_data = 0.0

    # Define a method to store the recevied data from the keyboard data subscriber and debug it
    def parse_keyboard_data(self, msg):
        self._key_data = msg.data
        self.get_logger().info(f"Received the keyboard data: {self._key_data}")

    

def main(args=None):
    rclpy.init(args=args)

    node = ActuateServo()
    rclpy.spin(node)

    rclpy.shutdown()