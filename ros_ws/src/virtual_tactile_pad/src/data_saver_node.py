#!/usr/bin/env python3
import rospy
from virtual_tactile_pad.msg import ContactForce
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
import os
import csv
from datetime import datetime

class DataSaver:
    def __init__(self):
        rospy.init_node('data_saver', anonymous=True)
        
        self.use_optitrack = rospy.get_param('~use_optitrack', False)
        self.use_ft_sensor = rospy.get_param('~use_ft_sensor', False)
        self.use_panda = rospy.get_param('~use_panda', False)
            
        self.data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'data', 'raw')
        os.makedirs(self.data_dir, exist_ok=True)
        self.filename = os.path.join(self.data_dir, f'data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
        
        headers = ['timestamp']
        if self.use_ft_sensor:
            headers.extend([
                'ft_x', 'ft_y', 'ft_z', 'ft_force_x', 'ft_force_y', 'ft_force_z',
                'ft_wrench_force_x', 'ft_wrench_force_y', 'ft_wrench_force_z',
                'ft_wrench_torque_x', 'ft_wrench_torque_y', 'ft_wrench_torque_z'
            ])

        if self.use_panda:
            headers.extend([
                'panda_x', 'panda_y', 'panda_z', 
                'panda_force_x', 'panda_force_y', 'panda_force_z',
                'panda_wrench_force_x', 'panda_wrench_force_y', 'panda_wrench_force_z',
                'panda_wrench_torque_x', 'panda_wrench_torque_y', 'panda_wrench_torque_z'
            ])
            headers.extend([f'tau_ext_{i}' for i in range(7)])
            headers.extend([f'jacobian_{i}_{j}' for i in range(6) for j in range(7)])
        
        if self.use_optitrack:
            headers.extend([
                'opti_x', 'opti_y', 'opti_z',
                'opti_orientation_x', 'opti_orientation_y', 
                'opti_orientation_z', 'opti_orientation_w'
            ])
        
        self.latest_ft_data = None if self.use_ft_sensor else []
        self.latest_ft_wrench = None if self.use_ft_sensor else []
        self.latest_panda_data = None if self.use_panda else []
        self.latest_panda_wrench = None if self.use_panda else []
        self.latest_pose_data = None if self.use_optitrack else []
        self.latest_tau_ext = None if self.use_panda else []
        self.latest_jacobian = None if self.use_panda else []

        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(headers)
        
        self.write_interval = 1.0 / 80.0  # 80 Hz

        if self.use_ft_sensor:
            self.ft_wrench_sub = rospy.Subscriber(
                "/ft_process_node/wrench",
                WrenchStamped,
                self.ft_wrench_callback
            )
            self.ft_sub = rospy.Subscriber(
                "/ft_process_node/contact_force",
                ContactForce,
                self.ft_contact_callback
            )
        
        if self.use_panda:
            self.panda_wrench_sub = rospy.Subscriber(
                "/panda_process_node/wrench",
                WrenchStamped,
                self.panda_wrench_callback
            )
            self.panda_sub = rospy.Subscriber(
                "/panda_process_node/contact_force",
                ContactForce,
                self.panda_contact_callback
            )
            self.tau_ext_sub = rospy.Subscriber(
                "/panda_process_node/tau_ext",
                Float64MultiArray,
                self.tau_ext_callback
            )
            self.jacobian_sub = rospy.Subscriber(
                "/panda_process_node/jacobian",
                Float64MultiArray,
                self.jacobian_callback
            )
            
        if self.use_optitrack:
            self.opti_sub = rospy.Subscriber(
                '/vrpn_client_node/marker_pad/pose_from_base_pad',
                PoseStamped,
                self.optitrack_data_callback
            )
            
        rospy.Timer(rospy.Duration(self.write_interval), self.write_data)
        rospy.loginfo(f"Saving data to: {self.filename}")
        rospy.loginfo("Data logger initialized")

    def ft_wrench_callback(self, msg):
        if self.use_ft_sensor:
            self.latest_ft_wrench = [
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
            ]

    def ft_contact_callback(self, msg):
        if self.use_ft_sensor:
            self.latest_ft_data = [
                msg.position.x, msg.position.y, msg.position.z,
                msg.force.x, msg.force.y, msg.force.z
            ]

    def panda_wrench_callback(self, msg):
        if self.use_panda:
            self.latest_panda_wrench = [
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
            ]

    def panda_contact_callback(self, msg):
        if self.use_panda:
            self.latest_panda_data = [
                msg.position.x, msg.position.y, msg.position.z,
                msg.force.x, msg.force.y, msg.force.z
            ]

    def tau_ext_callback(self, msg):
        if self.use_panda:
            self.latest_tau_ext = msg.data

    def jacobian_callback(self, msg):
        if self.use_panda:
            self.latest_jacobian = msg.data

    def optitrack_data_callback(self, msg):
        if self.use_optitrack:
            self.latest_pose_data = [
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y, 
                msg.pose.orientation.z, msg.pose.orientation.w
            ]

    def write_data(self, event=None):
        current_time = rospy.get_time()
        
        conditions = []
        if self.use_ft_sensor:
            conditions.extend([
                self.latest_ft_data is not None,
                self.latest_ft_wrench is not None
            ])
        if self.use_panda:
            conditions.extend([
                self.latest_panda_data is not None,
                self.latest_panda_wrench is not None,
                self.latest_tau_ext is not None,
                self.latest_jacobian is not None
            ])
        if self.use_optitrack:
            conditions.append(self.latest_pose_data is not None)
        
        if all(conditions):
            row_data = [current_time]
            
            if self.use_ft_sensor:
                row_data.extend(self.latest_ft_data)
                row_data.extend(self.latest_ft_wrench)
            
            if self.use_panda:
                row_data.extend(self.latest_panda_data)
                row_data.extend(self.latest_panda_wrench)
                row_data.extend(self.latest_tau_ext)
                row_data.extend(self.latest_jacobian)

            if self.use_optitrack:
                row_data.extend(self.latest_pose_data)
            
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