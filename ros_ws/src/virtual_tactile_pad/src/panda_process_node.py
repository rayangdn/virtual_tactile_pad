#!/usr/bin/env python3

import rospy
from franka_msgs.msg import FrankaState
from virtual_tactile_pad.msg import ContactForce
import numpy as np
import yaml
import os
import pinocchio as pin
import rospkg

# Get package path using rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('virtual_tactile_pad')

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

        # Initialize Pinocchio model
        urdf_path = os.path.join(package_path, 'urdf/panda_arm.urdf')  
        self.model_pin = pin.buildModelFromUrdf(urdf_path)
        self.data_pin = self.model_pin.createData()
        self.frame_id = self.model_pin.getFrameId("ati_net_gamma")

        # Initialize panda position relative to origin of the pad (front left)
        self.PANDA_POS = panda_pos
        
        # Get and validate calibration type from config file
        self.CALIBRATION_TYPE = 'static'  # Default calibration type
        if self.CALIBRATION_TYPE not in self.VALID_CALIBRATION_TYPES:
            rospy.logerr(f"Invalid calibration type: {self.CALIBRATION_TYPE}. Using default 'static'")
            self.CALIBRATION_TYPE = 'static'
        rospy.loginfo(f"Calibration type: {self.CALIBRATION_TYPE}")
        
        # Initialize calibration parameters for static calibration
        if self.CALIBRATION_TYPE == 'static':
            self.static_calibration_array = []
            self.static_calibration_complete = False
            self.static_calibration_count = 0
            self.static_calibration_offset = np.zeros(6)
            self.STATIC_CALIBRATION_SAMPLES = config['calibration']['static']['num_samples']
        
        # Initialize ROS publisher for contact force data
        self.contact_force_pub = rospy.Publisher('/panda_process_node/contact_force', ContactForce, queue_size=10)

        # Create subscriber to the franka_states topic
        self.state_sub = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            FrankaState,
            self.callback
        )
        self.timer = 0
        rospy.loginfo("Panda wrapper initialized")

    def normal_jacobian(self, q):
        pin.forwardKinematics(self.model_pin, self.data_pin, q)
        pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)
        J = pin.computeFrameJacobian(self.model_pin, self.data_pin, q, self.frame_id)

        R_tot = np.zeros((6,6))
        R_tot[0:3, 0:3] = self.data_pin.oMf[self.frame_id].rotation
        R_tot[3:, 3:] = self.data_pin.oMf[self.frame_id].rotation
        J = R_tot @ J

        return J
        
    def callback(self, msg):
        tau_ext = np.array(msg.tau_ext_hat_filtered)
        q = np.array(msg.q)
        
        # Process measurement based on calibration type
        if self.CALIBRATION_TYPE == 'static':
            if not self.static_calibration_complete:
                self.static_calibrate(tau_ext)
            else:
                tau_ext -= self.static_calibration_offset
                self.process_data(tau_ext, q)
        else:
            self.dynamic_calibrate(tau_ext)
            self.process_data(tau_ext, q)
            
    def static_calibrate(self, params):
        self.static_calibration_array.append(params)
        self.static_calibration_count += 1
        
        if self.static_calibration_count >= self.STATIC_CALIBRATION_SAMPLES:
            self.static_calibration_offset = np.mean(self.static_calibration_array, axis=0)
            self.static_calibration_complete = True
            rospy.loginfo(f"Static calibration complete, Offset: {self.static_calibration_offset}")
            
    def process_data(self, tau_ext, q): 
        J = self.normal_jacobian(q)  # Using Pinocchio-based Jacobian calculation
        wrench = self.estimate_external_wrench(J, tau_ext)
        self.timer += 1

        contact_pos = self.estimate_contact_point(wrench)
        force = wrench[:3]
        if (self.timer >= 500):
            print(f"q: {q}")
            print(f"tau_ext:{tau_ext}")
            self.timer = 0

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
        self.contact_force_pub.publish(msg)

    def dynamic_calibrate(self, measurement):
        rospy.logwarn("Dynamic calibration not yet implemented")

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
        estimated_point = p_c_d + self.PANDA_POS
        estimated_point[0] = config['pad']['dimensions']['x'] - estimated_point[0]
            
        return estimated_point

if __name__ == '__main__':
    try:
        PandaWrapper()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rospy.signal_shutdown("User interrupted")
        
        