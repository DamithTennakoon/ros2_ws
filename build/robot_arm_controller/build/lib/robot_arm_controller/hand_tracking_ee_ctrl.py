#!/usr/bin/env python3

# Objective: Parse through input hand tracking data from Unity and utilize it to move robot arm in coorinate control mode.
# Written by: Damith Tennakoon

# NOTES:
# - Branched off the "hand_coord_controller.py" ROS2 node
# - **Does not perform joint0-joint6 alignment (found issues with initialize implementation [used get_coords(), not get_angles()])

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
class HandTrackingEECtrl(Node):

    def __init__(self):
        super().__init__("hand_tracking_ee_ctrl")
        self.get_logger().info("INITIALIZING HAND TRACKING END EFFECTOR CONTROLLER NODE")

        # Initialize connection to robot arm
        self.get_logger().info("INITIALIZING CONNECTION TO ROBOT ARM") 
        self._mc = MyCobot("/dev/ttyACM0", 115200) # Instance of the MyCobot class
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
        self.get_logger().info("ROBOT ARM JOINT INITIALIZATION COMPLETE - STATUS [READY]")

        # Create Publisher/Subscriber objects
        self._publish_robot_pose = self.create_publisher(Float64MultiArray, 'robot_pose', 10) # Transmit the robot's pose for tracking/plotting 
        self._publish_robot_joint_angles = self.create_publisher(Float64MultiArray, 'robot_joint_angles', 10) # Transmit the robot's joints for 3D digital twin
        self.raw_data_subscriber = self.create_subscription(String, 'raw_input_data', self.parse_raw_data, 10) # Receive the string data from User Input Segment

        # Define variables for local data storage 
        self._hand_control_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Floating point list of hand control data
        self._cur_position = self._mc.get_coords() # [x, y, z, pitch, roll, yaw]
        self._incr_pos = 1.0 # Position increment for EE
        self._command_delay = 0.04 # Delay after transmitting motion command
        self._move_speed = 25 # Arm movement speed in mm/s
        self._robot_offset = 97 # Offset between the joint 0 and joint 6 on the xy-plane, in mm.
        self._t_unity_to_robot = np.array([
            [-1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ]) # Transfomration matrix from Unity coordainte system to Robot coordinate system

        # Create and excute callback functions
        self._move_robot_timer = self.create_timer(0.01, self.move_robot_arm)
    
    # Event Handler method - parse and store the HTC received data
    def parse_raw_data(self, msg):
        # Parse data if the received message is for HTC
        if (msg.data[:3] == "HTC"):
            split_string_list = msg.data.split(',') # Seperate the string using csv format
            # Convert and store the data 
            for i in range(len(self._hand_control_data)):
                self._hand_control_data[i] = float(split_string_list[i+1]) 
        else:
            for i in range(len(self._hand_control_data)):
                self._hand_control_data[i] = 0.0 # Zero the data
        # TEMP: Log the output data for testing comms
        self.get_logger().info(f"{self._hand_control_data}")

    # Callback method - move end effector of robot arm using cartesian coordinate control function
    def move_robot_arm(self):
        # Update position vector when hand control data is non-zero
        if ((math.sqrt(self._hand_control_data[0]**2 + self._hand_control_data[1]**2 + self._hand_control_data[2]**2)) > 0.0):
            unity_coords = np.array([self._hand_control_data[0], self._hand_control_data[1], self._hand_control_data[2]]) # Construct numpy array of unity unit vector
            robot_coords = np.dot(self._t_unity_to_robot, unity_coords) # Perform coordinate transformation - unity frame to robot frame
            #self._cur_position[0] += -1*(self._hand_control_data[0] * self._incr_pos) # Move right/left in the robot's adjusted coord frame
            #self._cur_position[1] += -1*(self._hand_control_data[2] * self._incr_pos) # Move forward/backward in the robot's adjusted coord frame
            #self._cur_position[2] += 1*(self._hand_control_data[1] * self._incr_pos) # Move up/down in the robot's adjusted coord frame
            self._cur_position[0] += robot_coords[0] # Move robot along its x-axis
            self._cur_position[1] += robot_coords[1] # Move robot along its y-axis
            self._cur_position[2] += robot_coords[2] # Move robot along its z-axis
            self._cur_position[5] = yaw_axis_alignment(self._cur_position, self._robot_offset)
            self._mc.send_coords(self._cur_position, self._move_speed, 1) # Transmit coordinate control command
            time.sleep(self._command_delay) # Delay to move arm to position
        else:
            self._cur_position = self._cur_position


# Create main method for looping the ROS node
def main(args=None):
    try:
        rclpy.init(args=args)
        node = HandTrackingEECtrl()
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
