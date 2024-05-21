#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time

class KeyboardCoordController(Node):

    def __init__(self):
        super().__init__("keyboard_coord_controller")
        self.get_logger().info("INITIALIZING DATA HANDLER NODE...")

        # Initialize connection to robot arm
        self._mc = MyCobot("/dev/ttyACM0", 115200)

        # Construct a publisher object to publish the robot's pose data to the topic "robot_pose"
        self._publish_robot_pose = self.create_publisher(Float64MultiArray, 'robot_pose', 10)

        # Construct a publisher object to publish the robot's joint angle data to the topic "robot_joint_angles"
        self._publish_robot_joint_angles = self.create_publisher(Float64MultiArray, 'robot_joint_angles', 10)
        
        # Define a subscriber node to lsiten for the raw input data topic
        self.raw_data_subscriber = self.create_subscription(String, 'raw_input_data', self.store_raw_data, 10)

        # Define a callback function to move robot arm
        self._move_robot_timer = self.create_timer(0.005, self.move_robot_arm)

        # Define a callback function to retrive the joint angles
        self._retrieve_joint_angles = self.create_timer(0.01, self.retrieve_joint_angles)

        # Define string object to store received data
        self._input_key = "NONE"

        # Define array to initialize and store robot arm's coordaintes
        self._cur_position = self._mc.get_coords() # [x, y, z, pitch, roll, yaw]

        # Define float object to store move value
        self._incr_pos = 0.5

        # Define float object to store joint 0 angle
        self._joint_0_angle = self._cur_position[0]

        # Define float object to store previos Joint #6 angle
        self._prev_joint_6_angle = self._cur_position[5] 

    # Define callback function to store topic data into internal variables
    def store_raw_data(self, msg):
        self._input_key = msg.data
        self.get_logger().info(f"ECHO: {msg.data}")

    # Define a callback function to translate the robot arm's end effector in coordinate space
    def move_robot_arm(self):
        # Update position vector
        if (self._input_key == "UpArrow"):
            self._cur_position[0] += self._incr_pos
        elif (self._input_key == "DownArrow"):
            self._cur_position[0] -= self._incr_pos
        elif (self._input_key == "RightArrow"):
            self._cur_position[1] -= self._incr_pos
        elif (self._input_key == "LeftArrow"):
            self._cur_position[1] += self._incr_pos
        elif (self._input_key == "N"):
            self._cur_position[2] += self._incr_pos
        elif (self._input_key == "M"):
            self._cur_position[2] -= self._incr_pos
        else:
            self._cur_position = self._cur_position

        # Align Joint #6 with Joint #1
        self._cur_position[5] = self._joint_0_angle - 5

        # Trasnmit UART message to robot arm
        self._mc.send_coords(self._cur_position, 60, 1)

    # Define a callback function to retrive and store the angle of joint 0
    def retrieve_joint_angles(self):
        # Define a Float32 object
        msg = Float64MultiArray()

        # Define a String object to store joint angle data
        joint_angles_msg = Float64MultiArray()
        
        # Store the joint angles as a local variable
        cur_angles = self._mc.get_angles()

        # Store the pose as a local variables
        cur_pose = self._mc.get_coords()

        # Store the current pose into the MultiArray object
        msg.data = cur_pose

        # Convert the current joint angles into the Multiarray object
        joint_angles_msg.data = cur_angles
        
        # Store angle of joint 0
        self._joint_0_angle = cur_angles[0]

        # Publish the pose data to the 'robot_pose' topic
        self._publish_robot_pose.publish(msg)

        # Publish the joint angle data to the 'robot_joint_angles' topic
        self._publish_robot_joint_angles.publish(joint_angles_msg)

        # Log the angle
        self.get_logger().info(f"Joint 0 angle: {self._joint_0_angle}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCoordController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
