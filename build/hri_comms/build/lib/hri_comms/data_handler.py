#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import String

class DataHandler(Node):

    def __init__(self):
        super().__init__("data_handler")
        self.get_logger().info("INITIALIZING DATA HANDLER NODE...")
        
        # Define a subscriber node to lsiten for the raw input data topic
        self.raw_data_subscriber = self.create_subscription(String, 'raw_input_data', self.store_raw_data, 10)

    # Define callback function to echo topic messages
    def store_raw_data(self, msg):
        self.get_logger().info(f"ECHO: {msg.data}")
        
def main(args=None):
    rclpy.init(args=args)
    node = DataHandler()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
