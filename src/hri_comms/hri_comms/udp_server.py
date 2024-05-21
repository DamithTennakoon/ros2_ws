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
        self._server_ip = '130.63.228.232'
        self._client_ip = []

        # Initialize UDP server
        self._udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_server.bind((self._server_ip, self._server_port))
        self.get_logger().info("UDP server awaiting client connection...")

        # Construct a publisher object to publish the received data to the topic "raw_input_data"
        self._publish_data = self.create_publisher(String, 'raw_input_data', 10)

        # Consutrct a subsriber object to subscribe to robot arm's joint angles data from the topic "robot_joint_angles" 
        self._robot_joint_angles_data = self.create_subscription(Float64MultiArray, 'robot_joint_angles', self.parse_joint_angle_data, 10)

        # Construct a timer to execute the rx_tx_server callback function every millisecond
        self.create_timer(0.001, self.rx_tx_server)
        
    def rx_tx_server(self):
        # Store the received data and the client ip adress and decode the received message
        self._rx_data, self._client_ip = self._udp_server.recvfrom(self._buffer_size)
        self._rx_data = self._rx_data.decode('utf-8')

        # Debug to the console the received data
        self.get_logger().info(f"RX: {self._rx_data}")

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
        self._tx_data = ','.join(joint_angles_string)


def main (args=None):
    rclpy.init(args=args)
    node = UdpServer()
    rclpy.spin(node)
    rclpy.shutdown()

'''
# Import libs
import socket
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
import time

# Initialize robot arm paramaters
mc = MyCobot("/dev/ttyACM0", 115200)
#mc.send_angles([0, 0, 0, 0, 0, 0], 40)
time.sleep(5)
position = mc.get_coords() # [x, y, z]

# Initialize UDP server parameters
dataTX = "ROS2"
dataRX = ""
bufferSize = 1024 
ServerPort = 2222
ServerIP = '130.63.230.194'

# Initialize UDP server
RPIsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
RPIsocket.bind((ServerIP, ServerPort))
print("Waiting for client to connect...")

# Continous TX/RX
while (True):
    # Store and decode received message + IP
    dataRX, clientIP = RPIsocket.recvfrom(bufferSize)
    dataRX = dataRX.decode('utf-8')
    #print("RX: ", dataRX)

    # Update position vector
    if (dataRX == "FORWARD"):
        position[0] += 0.1
    elif (dataRX == "BACKWARD"):
        position[0] -= 0.1
    elif (dataRX == "RIGHT"):
        position[1] += 0.1
    elif (dataRX == "LEFT"):
        position[1] -= 0.1
    elif (dataRX == "UP"):
        position[2] += 0.1
    elif (dataRX == "DOWN"):
        position[2] -= 0.1
    else:
        position = position

    #mc.send_coords([round(position[0], 1), round(position[1], 1), position[2], position[3], position[4], position[5]], 30, 0)
    mc.send_coords(position, 30, 1)
    print(position)
    time.sleep(0.001)

    # Transmit data to server
    bytesDataTX = dataTX.encode('utf-8')
    RPIsocket.sendto(bytesDataTX, clientIP)
'''
