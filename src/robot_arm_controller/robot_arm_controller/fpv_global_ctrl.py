#!/usr/bin/env python3

# Objective: Move the end effector pose of the robot arm in the first person view and third person view controll modes.
# Written by: Damith Tennakoon

# NOTES:

# Import ROS2 libraries
import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

# Import myCobot libraries
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD

# Import signal libraries
import time

# Import computational libraries
import math

# Import computational libraries
import math
import numpy as np

# Function - compute the yaw angle required to align end effector eith the robot's joint 0 motor 
def yaw_axis_alignment(current_pose, offset):
    p1_x = current_pose[0] # Parse the x,y coordinates
    p1_y = current_pose[1]
    beta = math.atan(-1*p1_x/p1_y) # Compute the immediate angle, radians
    p2_x = p1_x - offset * math.cos(beta) # Compute the x-component due to the offset
    p2_y = p1_y - offset * math.sin(beta) # Compute the y-component due to the offset
    yaw_angle_degrees = math.degrees(math.atan(-p2_x/p2_y)) + 90.0 # Compute actual immediate angle and account for offset angle, degrees
    return yaw_angle_degrees

# Construct Class
class FpvGlobalCtrl(Node):

    def __init__(self):
        super().__init__("fpv_global_ctrl")
        self.get_logger().info("INITIALIZING FPV GLOBAL END EFFECTOR CONTROLLER NODE")

# Create main method for looping the ROS node
def main(args=None):
    try:
        rclpy.init(args=args)
        node = FpvGlobalCtrl()
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
