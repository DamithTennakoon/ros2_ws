#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time

# Import computational libraries
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

# Function - compute the yaw angle required to align end effector eith the robot's joint 0 motor 
def yaw_axis_alignment(current_pose, offset):
    p1_x = current_pose[0] # Parse the x,y coordinates
    p1_y = current_pose[1]
    beta = math.atan(-1*p1_x/p1_y) # Compute the immediate angle, radians
    p2_x = p1_x - offset * math.cos(beta) # Compute the x-component due to the offset
    p2_y = p1_y - offset * math.sin(beta) # Compute the y-component due to the offset
    yaw_angle_degrees = math.degrees(math.atan(-p2_x/p2_y)) + 90.0 # Compute actual immediate angle and account for offset angle, degrees
    return yaw_angle_degrees

class GlobalRobotCtrl(Node):

    def __init__(self):
        super().__init__("global_robot_ctrl")
        self.get_logger().info("INITIALIZING GLOBAL ROBOT CTRL NODE...")

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
        self._move_robot_timer = self.create_timer(0.001, self.move_robot_arm) # DEFAULT: 0.005
        self._retrieve_joint_angles = self.create_timer(0.01, self.retrieve_joint_angles)

        # Define variables for local data storage 
        self._input_key = "NONE"
        self._cur_position = self._mc.get_coords() # [x, y, z, pitch, roll, yaw]
        self._incr_pos = 2.0 # DEF -> 0.5 -> 1 -> 1.3 ->LAST CHANGE 0.8
        self._joint_0_angle = self._cur_position[0]
        self._prev_joint_6_angle = self._cur_position[5] 
        self._target_position = [0.0, 0.0, 0.0]
        self._target_rotation = [0.0, 0.0, 0.0, 0.0] # qx, qy, qz, qw
        self._cur_position = self._mc.get_coords() # [x, y, z, pitch, roll, yaw]
        self._robot_offset = 97 # Offset between the joint 0 and joint 6 on the xy-plane, in mm.
        self._move_speed = 25 # Arm movement speed in mm/s
        self._command_delay = 0.04 # Delay after transmitting motion command

    # Define callback function to store topic data into internal variables
    def store_raw_data(self, msg):
        self._input_key = msg.data
        
        if (msg.data[:3] == "GRC"):
            split_string_list = msg.data.split(',') # Seperate the string using csv format
            # Convert and store the data 
            for i in range(len(self._target_position)):
                self._target_position[i] = float(split_string_list[i+1]) 
            for j in range(len(self._target_rotation)):
                self._target_rotation[j] = float(split_string_list[j+4])
                

    # Define a callback function to translate the robot arm's end effector in coordinate space
    def move_robot_arm(self):
        # Send position data to robot arm
        if (((self._target_position[0]**2)+(self._target_position[1]**2)+(self._target_position[2]**2)) > 0.0):
            self._cur_position[0] = self._target_position[2] * 1000
            self._cur_position[1] = self._target_position[0] * -1000
            self._cur_position[2] = self._target_position[1] * 1000
            self._cur_position[5] = yaw_axis_alignment(self._cur_position, self._robot_offset)
            r = R.from_quat(self._target_rotation)
            euler_angles = r.as_euler('xyz', degrees=True)  # returns roll, pitch, yaw in degrees
            roll, pitch, yaw = euler_angles
            # Logging
            #print(f"Target poistion in mm: {self._cur_position[0:4]}")
            #print(f"Target rotation: {self._target_rotation}")
            print(f"Euler Angles: {euler_angles}")
            # Serial communications
            #self._mc.send_coords(self._cur_position, self._move_speed, 1) # Execute coordinate control command
            time.sleep(self._command_delay) # Delay to move arm to position

    # Define a callback function to retrive and store the angle of joint 0
    def retrieve_joint_angles(self):
        # Retrieve joint angle data
        joint_angles_msg = Float64MultiArray()
        cur_angles = self._mc.get_angles()
        joint_angles_msg.data = cur_angles

        # Publish the data to the topic
        self._publish_robot_joint_angles.publish(joint_angles_msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = GlobalRobotCtrl()
        rclpy.spin(node)
    except Exception as main_err:
        print(f"Error initializing node: {str(main_err)}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("Node shutdown complete.")

if __name__ == '__main__':
    main()