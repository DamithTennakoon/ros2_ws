#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Create a class for the node
class MyNode(Node): # Name of class

    # Create a consturctor
    def __init__(self):
        super().__init__("first_node")# Actual name of the node that will show up in the graph

        # Create a timer for the timer_callback method
        self.create_timer(0.1, self.timer_callback)
        

    # Create a method in the node that will get called back in the super method
    def timer_callback(self):
        self.get_logger().info("Hello")


def main(args=None):
    rclpy.init(args=args) # Initiailizes ROS2 comms

    # Create a node
    node = MyNode()

    # Spin the node so it doesnt automaticllay shutdown - looping
    rclpy.spin(node)

    rclpy.shutdown() # Shuts down ROS2 comms

if __name__ == '__main__':
    main()