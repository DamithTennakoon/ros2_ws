#!/usr/bin/env python3

# Objective: Map the pitch and yaw angles transmitted from the HMD to the two servo motors, for real-time head tracking.
# Writtern by: Damith Tennakoon

# Notes:
# - Define the Node

# Import ROS2 libraries
import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

# Import signal libraries
import time

# Import computational libraries
import math
import numpy as np

# Construct class
class HeadTracking(Node):

    def __init__(self):
        super().__init__("head_tracking")
        self.get_logger().info("INITIALIZING HEAD TRACKING NODE")

# Create main method for looping the ROS node
def main(args=None):
    try:
        rclpy.init(args=args)
        node = HeadTracking()
        rclpy.spin(node)
    except Exception as main_err:
        print(f"ERROR INITIALIZING NODE: {str(main_err)}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("NODE SHUTDOWN COMPLETE")

# Execute the main function
if __name__ == '__main__':
    main()