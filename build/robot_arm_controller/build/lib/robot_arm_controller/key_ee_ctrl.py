#!/usr/bin/env python3

# Objective: Parse through input keyboard data from Unity and utilize it to move robot arm in coorinate control mode.
# Written by: Damith Tennakoon

# NOTES:
# - Branched off the "keyboard_coord_controller" node
# - Does not perform gripper control within this node

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
class KeyEECtrl(Node):

    def __init__(self):
        super().__init__("key_ee_ctrl")
        self.get_logger().info("INITIALIZING KEY END EFFECTOR CONTROLLER NODE")

        # Initialize connection to robot arm
        self.get_logger().info("INITIALIZING CONNECTION TO ROBOT ARM") 
        self._mc = MyCobot("/dev/ttyACM0", 115200) # Instance of the MyCobot class
        time.sleep(1)
        self._mc.init_gripper()
        self._mc.set_gripper_mode(0)
        time.sleep(1) 
        self.get_logger().info("CONNECTION ESTABLISHED")

        # Initialize robot arm movements
        self.get_logger().info("INITIALIZING ROBOT ARM JOINTS AND WORKSPACE")
        self._mc.set_color(255, 0, 0)
        time.sleep(0.5)
        self._mc.send_angles([110, 63.8, 38.67, -20, -88.59, 90], 10) # Move to initialize position 1 using joint controller method
        time.sleep(10)
        self._mc.send_coords([93, -120, 280, 180, 7, 95], 10, 1) # Move to initialize position 2 (start pose) using coordinate controller method
        time.sleep(10)
        self._mc.set_color(255, 255, 255)
        time.sleep(0.5)
        self._mc.set_gripper_state(0, 80) # Close gripper
        time.sleep(3)
        self._mc.set_gripper_state(1,80) # Open Gripper
        time.sleep(3)
        self.get_logger().info("ROBOT ARM JOINT INITIALIZATION COMPLETE - STATUS [READY]")

        # Create Publisher/Subscriber objects
        self._publish_robot_pose = self.create_publisher(Float64MultiArray, 'robot_pose', 10) # Transmit the robot's pose for tracking/plotting 
        self._publish_robot_joint_angles = self.create_publisher(Float64MultiArray, 'robot_joint_angles', 10) # Transmit the robot's joints for 3D digital twin
        self.raw_data_subscriber = self.create_subscription(String, 'raw_input_data', self.store_raw_data, 10) # Receive the string data from User Input Segment

        # Define variables for local data storage 
        self._input_key = "NONE" # Received string message of the keyboard input
        self._cur_position = self._mc.get_coords() # [x, y, z, pitch, roll, yaw]
        self._incr_pos = 1.0 # Position increment for EE
        self._command_delay = 0.04 # Delay after transmitting motion command
        self._move_speed = 25 # Arm movement speed in mm/s
        self._robot_offset = 97 # Offset between the joint 0 and joint 6 on the xy-plane, in mm.
        self._gripper_open = 100 # The maximum extension (open) value for the robot gripper
        self._gripper_close = 20 # The close value for the robot gripper
        self._gripper_speed = 50 # Speed of the gripper, in mm/s.
        self._gripper_delay = 1.0 # Delay after transmitting gripper command

        # Create/execute callback functions
        self._move_robot_timer = self.create_timer(0.01, self.move_robot_arm)


    # Event Handler method - store the raw User Input Segment's keyboard data, locally
    def store_raw_data(self, msg):
        self._input_key = msg.data

    # Callback method - move end effector of robot arm using cartesian coordinate control fuction
    def move_robot_arm(self):
        # Update position vector based on input key data
        if (self._input_key == "UpArrow"):
            self._cur_position[0] += self._incr_pos # Increment on x-axis
            self._cur_position[5] = yaw_axis_alignment(self._cur_position, self._robot_offset)
            self._mc.send_coords(self._cur_position, self._move_speed, 1) # Execute coordinate control command
            time.sleep(self._command_delay) # Delay to move arm to position
        elif (self._input_key == "DownArrow"):
            self._cur_position[0] -= self._incr_pos
            self._cur_position[5] = yaw_axis_alignment(self._cur_position, self._robot_offset)
            self._mc.send_coords(self._cur_position, self._move_speed, 1) 
            time.sleep(self._command_delay) 
        elif (self._input_key == "RightArrow"):
            self._cur_position[1] -= self._incr_pos
            self._cur_position[5] = yaw_axis_alignment(self._cur_position, self._robot_offset)
            self._mc.send_coords(self._cur_position, self._move_speed, 1) 
            time.sleep(self._command_delay) 
        elif (self._input_key == "LeftArrow"):
            self._cur_position[1] += self._incr_pos
            self._cur_position[5] = yaw_axis_alignment(self._cur_position, self._robot_offset)
            self._mc.send_coords(self._cur_position, self._move_speed, 1) 
            time.sleep(self._command_delay) 
        elif (self._input_key == "N"):
            self._cur_position[2] += self._incr_pos
            self._mc.send_coords(self._cur_position, self._move_speed, 1) 
            time.sleep(self._command_delay) 
        elif (self._input_key == "M"):
            self._cur_position[2] -= self._incr_pos
            self._mc.send_coords(self._cur_position, self._move_speed, 1) 
            time.sleep(self._command_delay) 
        elif (self._input_key == "Q"):
            self._mc.set_gripper_value(self._gripper_open, self._gripper_speed) # Open grippper command
            time.sleep(self._gripper_delay)
        elif (self._input_key == "W"):
            self._mc.set_gripper_value(self._gripper_close, self._gripper_speed) # Close grippper command
            time.sleep(self._gripper_delay)
        else:
            self._cur_position = self._cur_position

# Create main method for looping the ROS node
def main(args=None):
    try:
        rclpy.init(args=args)
        node = KeyEECtrl()
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
        
