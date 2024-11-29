#!/usr/bin/env python3
import rospy
from virtual_tactile_pad.msg import ContactForce
from geometry_msgs.msg import PoseStamped
import os
import csv
import yaml
from datetime import datetime

class DataSaver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('data_saver', anonymous=True)
        
        # Get parameters
        self.use_optitrack = rospy.get_param('~use_optitrack', False)
        self.use_ft_sensor = rospy.get_param('~use_ft_sensor', False)
        self.use_panda = rospy.get_param('~use_panda', False)
            
        # Set up data logging
        self.data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data', 'raw')
        os.makedirs(self.data_dir, exist_ok=True)
        
        self.filename = os.path.join(self.data_dir, f'data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
        
        # Initialize headers based on enabled data sources
        headers = ['timestamp']
        if self.use_ft_sensor:
            headers.extend(['ft_x', 'ft_y', 'ft_z', 'ft_force_x', 'ft_force_y', 'ft_force_z'])

        if self.use_panda:
            # Add panda-specific headers if needed
            headers.extend(['panda_x', 'panda_y', 'panda_z', 
                          'panda_force_x', 'panda_force_y', 'panda_force_z'])
        
        if self.use_optitrack:
            headers.extend(['opti_x', 'opti_y', 'opti_z', 
                            'opti_orientation_x', 'opti_orientation_y', 
                            'opti_orientation_z', 'opti_orientation_w'])
        
        # Initialize the CSV file with headers
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(headers)
        
        # Initialize data storage for synchronization
        self.latest_ft_data = None if self.use_ft_sensor else {}
        self.latest_pose_data = None if self.use_optitrack else {}
        self.latest_panda_data = None if self.use_panda else {}
        self.write_interval = 1.0 / 80.0  # 80 Hz write rate
        
        # Set up subscribers based on parameters
        if self.use_ft_sensor:
            self.ft_sub = rospy.Subscriber(
                "/ft_process_node/contact_force",
                ContactForce,
                self.ft_data_callback)
            rospy.loginfo("Subscribed to FT sensor contact force topic")
        
        if self.use_panda:
            self.panda_sub = rospy.Subscriber(
                "/ft_process_node/contact_force",
                ContactForce,
                self.panda_data_callback)
            rospy.loginfo("Panda data logging enabled")
            rospy.loginfo("Subscribed to Panda contact force topic")

        if self.use_optitrack:
            self.opti_sub = rospy.Subscriber(
                '/vrpn_client_node/marker_pad/pose_from_base_pad',
                PoseStamped,
                self.optitrack_data_callback)
            rospy.loginfo("Subscribed to Optitrack contact force topic")
            
        # Set up timer for periodic data writing
        rospy.Timer(rospy.Duration(self.write_interval), self.write_data)
        
        rospy.loginfo(f"Saving data to: {self.filename}")
        
        rospy.loginfo("Data logger initialized")

    def ft_data_callback(self, msg):
        if self.use_ft_sensor:
            self.latest_ft_data = {
                'ft_x': msg.position.x,
                'ft_y': msg.position.y,
                'ft_z': msg.position.z,
                'ft_force_x': msg.force.x,
                'ft_force_y': msg.force.y,
                'ft_force_z': msg.force.z
            }
            
    def panda_data_callback(self, msg):
        if self.use_panda:
            self.latest_panda_data = {
                'panda_x': msg.position.x,
                'panda_y': msg.position.y,
                'panda_z': msg.position.z,
                'panda_force_x': msg.force.x,
                'panda_force_y': msg.force.y,
                'panda_force_z': msg.force.z
            }

    def optitrack_data_callback(self, msg):
        if self.use_optitrack:
            self.latest_pose_data = {
                'opti_x': msg.pose.position.x,
                'opti_y': msg.pose.position.y,
                'opti_z': msg.pose.position.z,
                'opti_orientation_x': msg.pose.orientation.x,
                'opti_orientation_y': msg.pose.orientation.y,
                'opti_orientation_z': msg.pose.orientation.z,
                'opti_orientation_w': msg.pose.orientation.w
            }

    def write_data(self, event=None):
        current_time = rospy.get_time()
        
        # Check if we have all required data
        conditions = []
        if self.use_ft_sensor:
            conditions.append(self.latest_ft_data is not None)
        if self.use_panda:
            conditions.append(self.latest_panda_data is not None)
        if self.use_optitrack:
            conditions.append(self.latest_pose_data is not None)
        
        # Only write if we have all required data
        if all(conditions):
            row_data = [current_time]
            
            # Add FT sensor data if enabled
            if self.use_ft_sensor:
                row_data.extend([
                    self.latest_ft_data['ft_x'],
                    self.latest_ft_data['ft_y'],
                    self.latest_ft_data['ft_z'],
                    self.latest_ft_data['ft_force_x'],
                    self.latest_ft_data['ft_force_y'],
                    self.latest_ft_data['ft_force_z']
                ])
            # Add Panda data if enabled
            if self.use_panda:
                row_data.extend([
                    self.latest_panda_data['panda_x'],
                    self.latest_panda_data['panda_y'],
                    self.latest_panda_data['panda_z'],
                    self.latest_panda_data['panda_force_x'],
                    self.latest_panda_data['panda_force_y'],
                    self.latest_panda_data['panda_force_z']
                ])
            # Add OptiTrack data if enabled
            if self.use_optitrack:
                row_data.extend([
                    self.latest_pose_data['opti_x'],
                    self.latest_pose_data['opti_y'],
                    self.latest_pose_data['opti_z'],
                    self.latest_pose_data['opti_orientation_x'],
                    self.latest_pose_data['opti_orientation_y'],
                    self.latest_pose_data['opti_orientation_z'],
                    self.latest_pose_data['opti_orientation_w']
                ])
            
            with open(self.filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(row_data)

if __name__ == '__main__':
    try:
        DataSaver()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down data logger...")
    finally:
        rospy.signal_shutdown("User interrupted")