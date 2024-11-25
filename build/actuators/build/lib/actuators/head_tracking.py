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
import RPi.GPIO as GPIO
import time

# Import computational libraries
import math
import numpy as np

# Function - compute the pulse width for the MG996R servo given an input target angle in degrees
def angle_to_pulse_width(theta):
    offset_pulse = 7.5 # Offset pulse width (0-degrees)
    r_factor = 10/180 # pulse width (ms) per degree
    pwm = theta*r_factor + offset_pulse # Output pulse
    # Clamp the value between 2.5 and 12.5 
    if (pwm > 12.5):
        pwm = 12.5
    elif (pwm < 2.5):
        pwm = 2.5
    else:
        pwm = pwm
    return pwm

# Construct class
class HeadTracking(Node):

    def __init__(self):
        super().__init__("head_tracking")
        self.get_logger().info("INITIALIZING HEAD TRACKING NODE")

        # Initialize GPIO pins
        self._yaw_servo_pin = 18 # BCM 18 (translates to pin #12)
        GPIO.setmode(GPIO.BCM) # Set the GPIO pin mode to Broadcom SOC channel
        GPIO.setup(self._yaw_servo_pin, GPIO.OUT) # Set the GPIO pin as an output channel
        self._yaw_servo_pwm = GPIO.PWM(self._yaw_servo_pin, 50) # Set 50 Hz transmit frequency for BCM 18 (servo standard)
        self._yaw_servo_pwm.start(0) # Set the starting pulse width to 0 (not actuating)

        # Create Publisher/Subscriber objects
        self._raw_data_subscriber = self.create_subscription(String, 'raw_input_data', self.parse_raw_data, 10) # Receive the string data from the Raw Input Data Topic

        # Define variables for local data storage 
        self._hmd_pitch =  0.0 # HMD pitch rotation angle
        self._hmd_yaw = 0.0 # HMD yaw rotation angle

        # Define and execute callback functions
        self._actuate_servos_timer = self.create_timer(0.01, self.actuate_servos)

    # Invoke desctructor - invoked at ROS node shutdown to cleanup GPIO board
    def __del__(self):
        # Cleanup resources
        self.get_logger().info("CLEANING UP RESOURCES")
        if hasattr(self, '_yaw_servo_pwm'):
            self._yaw_servo_pin.stop()
        GPIO.cleanup()
        self.get_logger().info("GPIO CLEANUP COMPLETE")

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
        self._yaw_servo_pwm.ChangeDutyCycle(angle_to_pulse_width(self._hmd_yaw)) # Set the yaw servo angle to the HMD yaw angle

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
