#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time

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

    # Define callback function to store topic data into internal variables
    def store_raw_data(self, msg):
        self._input_key = msg.data
        
        if (msg.data[:3] == "GRC"):
            split_string_list = msg.data.split(',') # Seperate the string using csv format
            # Convert and store the data 
            for i in range(len(self._target_position)):
                self._target_position[i] = float(split_string_list[i+1]) 

        print(f"PARSED DATA: {self._target_position}")

    # Define a callback function to translate the robot arm's end effector in coordinate space
    def move_robot_arm(self):
        # Parse raw data into robot_coordinate data
        pass

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