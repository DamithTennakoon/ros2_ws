#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64MultiArray
import csv
import os

class CsvWriter(Node):
    def __init__(self):
        super().__init__("csv_writer")

        # Log initializing 
        self.get_logger().info("INITIALIZING CSV_WRITER NODE...")

        # Create a subscriber to listen for 'robot_pose' topic and executes a callback method (Event Based)
        self.raw_data_subscriber = self.create_subscription(Float64MultiArray, 'robot_pose', self.write_to_csv, 10)

        # Define a string object to store the file name
        self._file_name = '/home/rpi/Data Collection/Robot Pose Data/robot_pose.txt'

        # Open CSV file - replace if it exists
        with open(self._file_name, mode='w', newline='') as file:
            pass # Replaces file
        
        self.get_logger().info("CSV FILE INITIALIZED")

    # Define event based callback method to write Float64Array data into csv file
    def write_to_csv(self, msg):
        # Write data to file
        with open(self._file_name, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = CsvWriter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()