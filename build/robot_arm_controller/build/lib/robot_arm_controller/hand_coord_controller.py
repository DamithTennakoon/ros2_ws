#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time

class HandCoordController(Node):
    def __init__(self):
        super().__init__("hand_coord_controller")
        
        # Debug initializations for node
        self.get_logger().info("INITIALIZING HAND COORD CONTROLLER NODE...")

        # Initialize connection to robot arm
        self._mc = MyCobot("/dev/ttyACM0", 115200)

        # Construct a publisher object to publish the received data to the topic "raw_input_data"
        self._publish_robot_pose = self.create_publisher(Float64MultiArray, 'robot_pose', 10)

        # Define subscriber node to listen for the raw input data topic
        self.raw_data_subscriber = self.create_subscription(String, 'raw_input_data', self.parse_raw_data, 10)

        # Define a callback function to retrive the joint angles
        self._retrieve_joint_angles = self.create_timer(0.01, self.retrieve_joint_angles)

        # Define a callback function to move robot arm
        self._move_robot_timer = self.create_timer(0.005, self.move_robot_arm)

        # Define variables
        self._rx_string = "NONE"

        # Define a 7-float array to store the Hand Control Data
        self._hand_control_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Define array to initialize and store robot arm's coordaintes
        self._cur_position = self._mc.get_coords() # [x, y, z, pitch, roll, yaw]

        # Define float object to store move value
        self._incr_pos = 0.5

        # Define float object to store joint 0 angle
        self._joint_0_angle = self._cur_position[0]

    # Define a event based callback method to parse through received data
    def parse_raw_data(self, msg):
        # Check whether the received data is HTC or KC
        if (msg.data[:3] == "HTC"):
            # Store the received string into a class self variable
            self._rx_string = msg.data

            # Convert string into a a Float32 list
            split_string_list = self._rx_string.split(",")
            self._hand_control_data[0] = float(split_string_list[1])
            self._hand_control_data[1] = float(split_string_list[2])
            self._hand_control_data[2] = float(split_string_list[3])
            self._hand_control_data[3] = float(split_string_list[4])
            self._hand_control_data[4] = float(split_string_list[5])
            self._hand_control_data[5] = float(split_string_list[6])
            self._hand_control_data[6] = float(split_string_list[7])

        else:
            self._rx_string = "NONE"
        
        # TEMP: Debug received string data to console
        self.get_logger().info(f"Float32 list: {self._hand_control_data}")

    # Define a callback function to retrive and store the angle of joint 0
    def retrieve_joint_angles(self):
        # Store the pose as a local variables
        cur_pose = self._mc.get_coords()

        # Define a Float32 object
        msg = Float64MultiArray()

        # Store the current angles into the MultiArray object
        msg.data = cur_pose

        # Store angle of joint 0
        self._joint_0_angle = self._mc.get_angles()[0]

        # Publish the joint angles to the 'robot_pose' topic
        self._publish_robot_pose.publish(msg)

    # Define a callback function to translate the robot arm's end effector in coordinate space
    def move_robot_arm(self):
        # Update the robot arm positional list with hand control list
        self._cur_position[0] += (-1)*self._hand_control_data[0]*self._incr_pos # Cartesian X
        self._cur_position[1] += (-1)*self._hand_control_data[2]*self._incr_pos # Cartesian Y
        self._cur_position[2] += self._hand_control_data[1]*self._incr_pos # Cartesian Z

        # Align Joint #6 with Joint #1
        self._cur_position[5] = self._joint_0_angle - 5

        # Trasnmit UART message to robot arm
        self._mc.send_coords(self._cur_position, 60, 1)

def main(args=None):
    rclpy.init(args=args)
    node = HandCoordController()
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == '__main__':
    main()