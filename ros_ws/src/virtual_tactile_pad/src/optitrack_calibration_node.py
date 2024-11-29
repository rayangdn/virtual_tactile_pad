#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import csv
import os
from datetime import datetime

class PoseLogger:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_logger', anonymous=True)
        
        # Create data directory if it doesn't exist
        self.data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data', 'raw')
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Create CSV file with timestamp in name
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = os.path.join(self.data_dir, f'optitrack_calibration_data_{timestamp}.csv')
        
        # Initialize CSV file with headers
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y', 'z'])
        
        # Subscribe to the pose topic
        self.pose_sub = rospy.Subscriber(
            '/vrpn_client_node/marker_pad/pose_from_base_pad',
            PoseStamped,
            self.pose_callback)
        rospy.loginfo("Subscribed to Optitrack contact force topic")
        
        rospy.loginfo(f"Started logging pose data to: {self.csv_filename}")

    def pose_callback(self, msg):
        # Extract position data
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Save to CSV file
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([x, y, z])

if __name__ == '__main__':
    try:
        PoseLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass