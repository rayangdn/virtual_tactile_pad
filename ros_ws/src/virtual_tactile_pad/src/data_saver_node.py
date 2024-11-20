#!/usr/bin/env python3
import rospy
from virtual_tactile_pad.msg import ContactForce
from geometry_msgs.msg import PoseStamped
import os
import csv
import yaml
from datetime import datetime

class DataSaver:
    """
    A ROS node for logging Force/Torque data and OptiTrack pose data
    into a single CSV file.
    """
    def __init__(self):
        """Initialize the combined data logger node"""
        # Initialize ROS node
        rospy.init_node('data_saver', anonymous=True)
        
        # Load configuration
        config_path = os.path.join(os.path.dirname(__file__), '../config/config.yaml')
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        # Set up data logging
        self.data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data', 'raw')
        os.makedirs(self.data_dir, exist_ok=True)
        
        self.filename = os.path.join(self.data_dir, f'ft_optitrack_data.csv')
        
        # Initialize the CSV file with headers
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'timestamp',
                'contact_x', 'contact_y', 'contact_z',
                'force_x', 'force_y', 'force_z',
                'position_x', 'position_y', 'position_z',
                'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'
            ])
        
        # Initialize data storage for synchronization
        self.latest_ft_data = None
        self.latest_pose_data = None
        self.last_write_time = None
        self.write_interval = 1.0 / 100  # 100Hz write rate
        
        # Set up subscribers
        self.force_sub = rospy.Subscriber(
            "/ft_process_node/contact_force",
            ContactForce,
            self.ft_data_callback
        )
        
        self.pose_sub = rospy.Subscriber(
            '/vrpn_client_node/marker_pad/pose_from_base_pad',
            PoseStamped,
            self.optitrack_data_callback
        )
        
        # Set up timer for periodic data writing
        rospy.Timer(rospy.Duration(self.write_interval), self.write_data)
        
        rospy.loginfo(f"Saving  data to:")
        rospy.loginfo(f"Combined data: {self.filename}")

    def ft_data_callback(self, msg):
        """Store latest force data"""
        self.latest_ft_data = {
            'contact_x': msg.position.x,
            'contact_y': msg.position.y,
            'contact_z': msg.position.z,
            'force_x': msg.force.x,
            'force_y': msg.force.y,
            'force_z': msg.force.z
        }

    def optitrack_data_callback(self, msg):
        """Store latest pose data"""
        self.latest_pose_data = {
            'position_x': msg.pose.position.x,
            'position_y': msg.pose.position.y,
            'position_z': msg.pose.position.z,
            'orientation_x': msg.pose.orientation.x,
            'orientation_y': msg.pose.orientation.y,
            'orientation_z': msg.pose.orientation.z,
            'orientation_w': msg.pose.orientation.w
        }

    def write_data(self, event=None):
        """Periodically write synchronized data to CSV"""
        current_time = rospy.get_time()
        
        # Only write if we have both types of data
        if self.latest_ft_data and self.latest_pose_data:
            with open(self.filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    current_time,
                    self.latest_ft_data['contact_x'],
                    self.latest_ft_data['contact_y'],
                    self.latest_ft_data['contact_z'],
                    self.latest_ft_data['force_x'],
                    self.latest_ft_data['force_y'],
                    self.latest_ft_data['force_z'],
                    self.latest_pose_data['position_x'],
                    self.latest_pose_data['position_y'],
                    self.latest_pose_data['position_z'],
                    self.latest_pose_data['orientation_x'],
                    self.latest_pose_data['orientation_y'],
                    self.latest_pose_data['orientation_z'],
                    self.latest_pose_data['orientation_w']
                ])

    def run(self):
        """Run the node"""
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = DataSaver()
        logger.run()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down combined data logger...")
    finally:
        rospy.signal_shutdown("User interrupted")