#!/usr/bin/env python3

import rospy
from franka_msgs.msg import FrankaState
from virtual_tactile_pad.msg import ContactForce
import numpy as np
import yaml
import os

# Load configuration
config_path = os.path.join(os.path.dirname(__file__), '../config/config.yaml')
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)
    
class PandaWrapper:
    
    VALID_CALIBRATION_TYPES = config['calibration']['types']
    
    def __init__(self):
        
        # Initialize ROS node
        rospy.init_node('panda_process', anonymous=True)
        
        # Get and validate calibration type from config file
        self.CALIBRATION_TYPE = config['calibration']['use_calibration']
        if self.CALIBRATION_TYPE not in self.VALID_CALIBRATION_TYPES:
            rospy.logerr(f"Invalid calibration type: {self.CALIBRATION_TYPE}. Using default 'static'")
            self.CALIBRATION_TYPE = 'static'
        rospy.loginfo(f"Calibration type: {self.CALIBRATION_TYPE}")
        
        # Initialize calibration parameters for static calibration
        if self.CALIBRATION_TYPE == 'static':
            self.static_calibration_array = []  # Store calibration measurements
            self.static_calibration_complete = False  # Flag for calibration status
            self.static_calibration_count = 0  # Counter for calibration samples
            self.static_calibration_offset = np.zeros(6)  # Calibration offset vector
            self.STATIC_CALIBRATION_SAMPLES = config['calibration']['static']['num_samples']  # Number of samples for calibration
        
        # Initialize ROS publisher for contact force data
        self.contact_force_pub = rospy.Publisher('/panda_process_node/contact_force', ContactForce, queue_size=10)

        # Create subscriber to the franka_states topic
        self.state_sub = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            FrankaState,
            self.callback
        )
        
        rospy.loginfo("Panda wrapper initialized")

    def callback(self, msg):
        measurement = np.array([
            msg.tau_ext_hat_filtered, # External torque (6x1)
            msg.tau_J, # Joint torque (7x1)
            msg.q, # Joint position (7x1)
            msg.dq # Joint velocity (7x1)
        ])
        
        # Example: Print some information
        rospy.loginfo(f"Received measurement: {measurement}")
        
        # Process measurement based on calibration type
        if self.CALIBRATION_TYPE == 'static':
            if not self.static_calibration_complete:
                self.static_calibrate(measurement)
            else:
                measurement[:6] -= self.static_calibration_offset
                self.process_data(measurement)
        else:
            self.dynamic_calibrate(measurement)
            self.process_data(measurement)
            
    def static_calibrate(self, measurement):
        self.static_calibration_array.append(measurement[:6])
        self.static_calibration_count += 1
        
        if self.static_calibration_count >= self.STATIC_CALIBRATION_SAMPLES:
            self.static_calibration_offset = np.mean(self.static_calibration_array, axis=0)
            self.static_calibration_complete = True
            rospy.loginfo("Static calibration complete")
            
    def process_data(self, measurement): 
        rospy.logwarn("Dynamic calibration not yet implemented")
    
    def dynamic_calibrate(self, measurement):
        rospy.logwarn("Dynamic calibration not yet implemented")
    
if __name__ == '__main__':
    try:
        panda_sensor = PandaWrapper()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rospy.single_shutdown("User interrupted")