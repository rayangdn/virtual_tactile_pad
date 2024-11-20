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
        self.timer =0
        rospy.loginfo("Panda wrapper initialized")

    def callback(self, msg):

        tau_ext = np.array(msg.tau_ext_hat_filtered)# External torque (7x1)
        q =  np.array(msg.q) # Joint position (7x1)
        
        # Process measurement based on calibration type
        if self.CALIBRATION_TYPE == 'static':
            if not self.static_calibration_complete:
                self.static_calibrate(tau_ext)
            else:
                tau_ext -= self.static_calibration_offset
                self.process_data(tau_ext, q)
        else:
            self.dynamic_calibrate(measurement)
            self.process_data(measurement)
            
    def static_calibrate(self, params):
        self.static_calibration_array.append(params)
        self.static_calibration_count += 1
        
        if self.static_calibration_count >= self.STATIC_CALIBRATION_SAMPLES:
            self.static_calibration_offset = np.mean(self.static_calibration_array, axis=0)
            self.static_calibration_complete = True
            rospy.loginfo(f"Static calibration complete, Offset: {self.static_calibration_offset}")
            
    def process_data(self, tau_ext, q): 
        J = self.calculate_jacobian(q)
        wrench = self.estimate_external_wrench(J, tau_ext)
        self.timer+=1
        if (self.timer >= 500):
            print(wrench)
            self.timer=0

    def dynamic_calibrate(self, measurement):
        rospy.logwarn("Dynamic calibration not yet implemented")
    
    def get_panda_dh_params(self):
        return np.array([
            [0,      0,      0.333,  0],
            [0,      -np.pi/2, 0,      0],
            [0,      np.pi/2,  0.316,  0],
            [0.0825, np.pi/2,  0,      0],
            [-0.0825, -np.pi/2, 0.384,  0],
            [0,      np.pi/2,  0,      0],
            [0.088,  np.pi/2,  0,      0]
        ])
    
    def transform_matrix(self, a, alpha, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,          np.sin(alpha),             np.cos(alpha),            d],
            [0,          0,                      0,                     1]
        ])

    def calculate_jacobian(self, q):
        dh = self.get_panda_dh_params()
        
        # Initialize transformation matrices
        T = np.eye(4)
        z_axes = []
        positions = []
        
        # Calculate forward kinematics for each joint
        for i in range(7):
            # Update DH parameters with current joint angle
            current_dh = dh[i].copy()
            current_dh[3] = q[i]
            
            # Calculate transformation matrix
            Ti = self.transform_matrix(*current_dh)
            T = T @ Ti
            
            # Store z-axis and position for Jacobian calculation
            z_axes.append(T[:3, 2])
            positions.append(T[:3, 3])
        
        # End effector position
        p_ee = positions[-1]
        
        # Initialize Jacobian matrix
        J = np.zeros((6, 7))
        
        # Calculate Jacobian columns
        for i in range(7):
            if i == 0:
                p_i = np.zeros(3)
            else:
                p_i = positions[i-1]
                
            # Linear velocity component
            J[:3, i] = np.cross(z_axes[i], (p_ee - p_i))
            
            # Angular velocity component
            J[3:, i] = z_axes[i]
        
        return J

    def estimate_external_wrench(self, J, residuals):

        # Calculate pseudoinverse of Jacobian transpose
        J_pinv = np.linalg.pinv(J.T)
        
        # Project residuals to get wrench (force/moment)
        wrench = J_pinv @ residuals
                    
        return wrench

if __name__ == '__main__':
    try:
        panda_sensor = PandaWrapper()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rospy.signal_shutdown("User interrupted")