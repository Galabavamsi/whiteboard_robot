#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import os
import time
import math

class WhiteboardCleaner(Node):
    """
    This ROS 2 node controls the Cartesian robot and logs performance data.
    It generates 'cleaning_data.csv' for the final report.
    """
    def __init__(self):
        super().__init__('whiteboard_cleaner')
        self.get_logger().info('WhiteboardCleaner Node started.')

        # Publishers for JointGroup controllers (Standard for Jazzy)
        self.pub_x = self.create_publisher(Float64MultiArray, '/x_position_controller/commands', 10)
        self.pub_y = self.create_publisher(Float64MultiArray, '/y_position_controller/commands', 10)
        self.pub_duster = self.create_publisher(Float64MultiArray, '/duster_velocity_controller/commands', 10)

        # Robot & Board Dimensions
        self.board_size = 0.90  # 90 cm
        self.duster_radius = 0.15 # 15 cm radius
        self.step_size = 0.20  # 20 cm step (Overlap = 30cm - 20cm = 10cm)
        self.offset = self.duster_radius # Robot center offset from edge
        self.duster_speed = 5.0 # rad/s

        # Generate Path
        self.waypoints = self.generate_raster_scan()
        self.current_waypoint_index = 0
        self.is_cleaning = False
        self.start_time = None

        # Data Logging Setup
        self.csv_filename = 'cleaning_data.csv'
        self.init_csv_file()

        # Control Timers
        # 1. Fast timer for continuous duster velocity (keeps it spinning)
        self.spin_timer = self.create_timer(0.1, self.duster_spin_callback)
        # 2. Slower timer for trajectory waypoints (gives robot time to move)
        self.traj_timer = self.create_timer(2.5, self.trajectory_callback) 

        self.get_logger().info('Ready. The robot will start cleaning automatically.')

    def init_csv_file(self):
        """Creates the CSV file and writes the header row."""
        # Saves to the folder where you run the command (usually ~/ros2_ws)
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'X_Pos', 'Y_Pos', 'Duster_Speed'])
        self.get_logger().info(f"Data logging initialized to {self.csv_filename}")

    def log_data(self, x, y, speed):
        """Appends the current robot state to the CSV file."""
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds
        
        # Calculate elapsed time in seconds
        current_time = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
        
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            # Log: Time (s), X (m), Y (m), Speed (rad/s)
            writer.writerow([f"{current_time:.2f}", f"{x:.3f}", f"{y:.3f}", f"{speed:.1f}"])

    def generate_raster_scan(self):
        """Generates (x, y) coordinates for a lawnmower coverage path."""
        waypoints = []
        y = self.offset
        sweep_right = True
        
        # Loop up the Y-axis
        while y <= self.board_size - self.offset + 0.01:
            x_start = self.offset
            x_end = self.board_size - self.offset
            
            if sweep_right:
                # Left -> Right
                waypoints.append((x_start, y))
                waypoints.append((x_end, y))
            else:
                # Right -> Left
                waypoints.append((x_end, y))
                waypoints.append((x_start, y))
                
            y += self.step_size
            sweep_right = not sweep_right

        # Add stop signal
        waypoints.append((-1, -1)) 
        return waypoints

    def duster_spin_callback(self):
        """Keeps the duster spinning."""
        target_speed = self.duster_speed if self.is_cleaning else 0.0
        msg = Float64MultiArray(data=[target_speed])
        self.pub_duster.publish(msg)

    def trajectory_callback(self):
        """Moves to the next waypoint and logs data."""
        
        # Start the clock on the first move
        if not self.is_cleaning and self.current_waypoint_index == 0:
            self.is_cleaning = True
            self.start_time = self.get_clock().now().nanoseconds

        if self.current_waypoint_index < len(self.waypoints):
            x, y = self.waypoints[self.current_waypoint_index]
            
            # Check for end of path
            if x == -1 and y == -1:
                self.is_cleaning = False
                self.get_logger().info(f'Cleaning Finished. Data saved to {self.csv_filename}')
                self.destroy_timer(self.traj_timer)
                return
            
            self.get_logger().info(f'Moving to Waypoint {self.current_waypoint_index + 1}: X={x:.2f}, Y={y:.2f}')
            
            # Send Position Commands
            self.pub_x.publish(Float64MultiArray(data=[x]))
            self.pub_y.publish(Float64MultiArray(data=[y]))
            
            # Log Data for Report
            self.log_data(x, y, self.duster_speed)
            
            self.current_waypoint_index += 1

def main(args=None):
    rclpy.init(args=args)
    cleaner_node = WhiteboardCleaner()
    
    try:
        rclpy.spin(cleaner_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown to avoid errors
        if rclpy.ok():
            cleaner_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()