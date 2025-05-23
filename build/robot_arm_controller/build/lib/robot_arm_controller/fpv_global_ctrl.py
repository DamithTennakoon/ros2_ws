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

        # Initialize connection to robot arm
        self.get_logger().info("INITIALIZING CONNECTION TO ROBOT ARM") 
        self._mc = MyCobot("/dev/ttyACM0", 115200) # Instance of the MyCobot class
        time.sleep(1)
        self.get_logger().info("CONNECTION ESTABLISHED")

        # Initialize robot arm movements
        self.get_logger().info("INITIALIZING ROBOT ARM JOINTS AND WORKSPACE")
        self._mc.set_color(255, 0, 0)
        time.sleep(0.5)
        self._mc.send_coords([93, -120, 280, 180, 7, 95], 10, 1) # Move to initialize position 2 (start pose) using coordinate controller method
        time.sleep(5)
        self._mc.set_color(255, 255, 255)
        time.sleep(0.5)
        self.get_logger().info("ROBOT ARM JOINT INITIALIZATION COMPLETE - STATUS [READY]")

        # Create Publisher/Subscriber objects
        self._publish_robot_pose = self.create_publisher(Float64MultiArray, 'robot_pose', 10)
        self._publish_robot_joint_angles = self.create_publisher(Float64MultiArray, 'robot_joint_angles', 10)
        self.raw_data_subscriber = self.create_subscription(String, 'raw_input_data', self.store_raw_data, 10)

        # Create/execute callback functions
        self._move_robot_timer = self.create_timer(0.1, self.move_robot_arm) # DEFAULT: 0.1
        self._retrieve_joint_angles = self.create_timer(0.01, self.retrieve_joint_angles)

        # Define variables for local data storage 
        self._target_position = [0.0, 0.0, 0.0]
        self._target_rotation = [0.0, 0.0, 0.0, 0.0] # qx, qy, qz, qw
        self._cur_position = self._mc.get_coords() # [x, y, z, pitch, roll, yaw]
        self._prev_gripper_state = True # Initialize the previous gripper state to be "open"
        self._cur_gripper_state = self._prev_gripper_state # Initialize the current gripper state to the previous - stops overlapping signals

# Define callback function to store topic data into internal variables
    def store_raw_data(self, msg):        
        # Parse for First Person Control
        if (msg.data[:3] == "HTC"):
            split_string_list = msg.data.split(',') # Seperate the string using csv format
            # Convert and store the data 
            for i in range(len(self._target_position)):
                self._target_position[i] = float(split_string_list[i+1])
                # Cast and handle the gripper control values
                if (float(split_string_list[i+6]) == 1.0): 
                    self._cur_gripper_state = True # Set the current gripper state to "open"
                else:
                    self._cur_gripper_state = False # Set the current gripper state to "close"

        # Parse for Global Robot Control
        if (msg.data[:3] == "GRC"):
            split_string_list = msg.data.split(',') # Seperate the string using csv format
            # Convert and store the data 
            for i in range(len(self._target_position)):
                self._target_position[i] = float(split_string_list[i+1]) 
            for j in range(len(self._target_rotation)):
                self._target_rotation[j] = float(split_string_list[j+4])


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
