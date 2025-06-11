#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import socket

class UdpServer(Node):

    def __init__(self):
        super().__init__("udp_server")

        # Initialize UDP server parameters
        self._rx_data = ""
        self._tx_data = "ROS server"
        self._buffer_size = 1024
        self._server_port = 2222
        self._server_ip = '130.63.213.137'
        self._client_ip = []

        # Initialize UDP server
        self._udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_server.bind((self._server_ip, self._server_port))
        self.get_logger().info("UDP server awaiting client connection...")

        # Construct a publisher object to publish the received data to the topic "raw_input_data"
        self._publish_data = self.create_publisher(String, 'raw_input_data', 10)

        # Consutrct a subsriber object to subscribe to robot arm's joint angles data from the topic "robot_joint_angles" 
        self._robot_joint_angles_data = self.create_subscription(Float64MultiArray, 'robot_joint_angles', self.parse_joint_angle_data, 10)
        self._robot_pose_data = self.create_subscription(Float64MultiArray, 'robot_pose', self.parse_pose_data, 10)

        # Construct a timer to execute the rx_tx_server callback function every millisecond
        self.create_timer(0.001, self.rx_tx_server)

        # Define user parameters
        self._pose_datacode = "POSE"
        
    def rx_tx_server(self):
        # Store the received data and the client ip adress and decode the received message
        self._rx_data, self._client_ip = self._udp_server.recvfrom(self._buffer_size)
        self._rx_data = self._rx_data.decode('utf-8')

        # Debug to the console the received data
        #self.get_logger().info(f"RX: {self._rx_data}")
        #self.get_logger().info(f"TX: {self._tx_data}")

        # Transmit a message to the client
        self._udp_server.sendto(self._tx_data.encode('utf-8'), self._client_ip)

        # Publish raw input data
        msg = String()
        msg.data = self._rx_data
        self._publish_data.publish(msg)

    # Define method to convert Float64MultiArray data into a string message
    def parse_joint_angle_data(self, msg):
        # Convert floating point values into a string value
        joint_angles_string = [str(value) for value in msg.data]

        # Concatonate string array into a single variable
        #self._tx_data = ','.join(joint_angles_string)

    # Define a method to convert robot pose topic data into a transmittable string
    def parse_pose_data(self, msg):
        pose_string = [str(value) for value in msg.data] # Convert float values to a stringed array
        pose_string.insert(0, self._pose_datacode) # Insert the pose data code
        self._tx_data = ','.join(pose_string) # Set the transmit message variable to the pose data string seperated with a comma 
        self.get_logger().info(f"UPDATED TX DATA: {self._tx_data}")

def main (args=None):
    rclpy.init(args=args)
    node = UdpServer()
    rclpy.spin(node)
    rclpy.shutdown()
