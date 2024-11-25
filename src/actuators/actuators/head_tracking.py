#!/usr/bin/env python3

# Objective: Map the pitch and yaw angles transmitted from the HMD to the two servo motors, for real-time head tracking.
# Writtern by: Damith Tennakoon

# Notes:
# - Define the Node: x
# - Debug the UDP server data: 

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

        # Create Publisher/Subscriber objects
        self._raw_data_subscriber = self.create_subscription(String, 'raw_input_data', self.parse_raw_data, 10) # Receive the string data from the Raw Input Data Topic

        # Define variables for local data storage 
        self._hmd_pitch =  0.0 # HMD pitch rotation angle
        self._hmd_yaw = 0.0 # HMD yaw rotation angle

        # Define and execute callback functions
        self._actuate_servos_timer = self.create_timer(0.01, self.actuate_servos)

    # Event Handler method - parse and store the HTC received data
    def parse_raw_data(self, msg):
        # Parse data if the received message is for HTC
        if (msg.data[:3] == "HTC"):
            split_string_list = msg.data.split(',') # Seperate the string using csv format
            # Cast data type to float and store data
            self._hmd_pitch = float(split_string_list[8]) # Index is i+1 due to index[0] value
            self._hmd_yaw = float(split_string_list[9])

    # Callback method - actuate the servo motors using HMD angles
    def actuate_servos(self):
        self.get_logger().info(f"PITCH: {self._hmd_pitch}, YAW: {self._hmd_yaw}")

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