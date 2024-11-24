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
    
    def __init__(self, panda_pos=np.array([config['panda']['position']['x'],
                                           config['panda']['position']['y'],
                                           config['panda']['position']['z']], dtype=float)):
        
        # Initialize ROS node
        rospy.init_node('panda_process', anonymous=True)

        # Initialize panda position relative to origin of the pad (front left)
        self.PANDA_POS = panda_pos
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
        J = self.calculate_ee_jacobian(q)
        wrench = self.estimate_external_wrench(J, tau_ext)
        self.timer+=1

        contact_pos = self.estimate_contact_point(wrench)
        force = wrench[:3]
        if (self.timer >= 500):
            print(f"q: {q}")
            print(f"tau_ext:{tau_ext}")
            self.timer=0
        # Create and publish ContactForce message
        msg = ContactForce()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "pad"
        msg.position.x = contact_pos[0] 
        msg.position.y = contact_pos[1]
        msg.position.z = 0.0  # Assume contact point is on the pad
        msg.force.x = force[0]
        msg.force.y = force[1]
        msg.force.z = force[2]
        #self.contact_force_pub.publish(msg)
        

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

    def calculate_ee_jacobian(self, q):
        dh = self.get_panda_dh_params()
    
        # Initialize
        T = np.eye(4)
        z_axes = []
        positions = []
        
        # Forward kinematics
        for i in range(7):
            current_dh = dh[i].copy()
            current_dh[3] = q[i]
            Ti = self.transform_matrix(*current_dh)
            T = T @ Ti
            z_axes.append(T[:3, 2])
            positions.append(T[:3, 3])
        
        # End effector position and rotation
        R_ee = T[:3, :3]  # End-effector rotation matrix
        p_ee = positions[-1]
        
        # Initialize Jacobian
        J = np.zeros((6, 7))
        
        # Calculate Jacobian columns in base frame
        for i in range(7):
            p_i = np.zeros(3) if i == 0 else positions[i-1]
            J[:3, i] = np.cross(z_axes[i], (p_ee - p_i))
            J[3:, i] = z_axes[i]
        
        # Transform to end-effector frame
        R_block = np.block([
            [R_ee.T, np.zeros((3,3))],
            [np.zeros((3,3)), R_ee.T]
        ])
        J_ee = R_block @ J
        
        return J_ee

    def estimate_external_wrench(self, J, residuals):

        # Calculate pseudoinverse of Jacobian transpose
        J_pinv = np.linalg.pinv(J.T)
        
        # Project residuals to get wrench (force/moment)
        wrench = J_pinv @ residuals
                    
        return wrench

    def estimate_contact_point(self, measurement):
        force = measurement[:3]
        moment = measurement[3:]

        # Check if force is below threshold
        if np.sqrt(force[0]**2 + force[1]**2 + force[2]**2) < config['processing']['force_threshold']:
            return np.array([0.0, 0.0, 0.0])

        # Calculate skew-symmetric matrix from force vector
        S_f_ext = np.array([
            [0, -force[2], force[1]],
            [force[2], 0, -force[0]],
            [-force[1], force[0], 0]
        ])

        # Set up and solve system of equations for contact point
        C = np.array([[0, 0, 1]])
        A = np.vstack([-S_f_ext, C])
        b = np.concatenate([moment, [-self.PANDA_POS[2]]])

        p_c_d = np.linalg.lstsq(A, b, rcond=None)[0]
        estimated_point = p_c_d + self.PANDA_POS # Add sensor position to get contact point in pad frame
        estimated_point[0] = config['pad']['dimensions']['x'] - estimated_point[0] # Convert to pad frame
            
        return estimated_point

if __name__ == '__main__':
    try:
        PandaWrapper()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rospy.signal_shutdown("User interrupted")