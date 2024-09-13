#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time

class VariableGripperController(Node):
    def __init__(self):
        super().__init__("variable_gripper_controller")
        
        # Debug initializations for node
        self.get_logger().info("INITIALIZING HAND COORD CONTROLLER NODE...")

        # Initialize connection to robot arm
        self._mc = MyCobot("/dev/ttyACM0", 115200)
        self._mc.set_color(255, 255, 255)
        time.sleep(1)
        self._mc.send_angles([110, 63.8, 38.67, -20, -88.59, 90], 12) # Init angles
        time.sleep(5)
        self._mc.send_coords([93, -120, 280, 180, 7, 95], 12, 1)
        time.sleep(5)

        # Define subscriber node to listen for the raw input data topic
        self.raw_data_subscriber = self.create_subscription(String, 'raw_input_data', self.parse_raw_data, 10)

        # Define a callback function to retrive the joint angles
        self._control_gripper = self.create_timer(0.01, self.control_gripper)

        # Define variables
        self._rx_string = "NONE"
        self._gripper_pwm = 255

        # Define a 7-float array to store the Hand Control Data
        self._hand_control_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
        #self.get_logger().info(f"Float32 list: {self._hand_control_data}")
    
    def control_gripper(self):
        # 
        print("FUNCTIONAL")
        # Round and covert the variable gripper value into integer
        self._gripper_pwm = int((round(self._hand_control_data[6])))
        self.get_logger().info(f"PWM VALUE: {self._gripper_pwm}")

        # Set the gripper value
        self._mc.set_gripper_value(self._gripper_pwm, 80)
        time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = VariableGripperController()
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == '__main__':
    main()