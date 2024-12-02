#!/usr/bin/env python3

import rospy
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import WrenchStamped
from virtual_tactile_pad.msg import ContactForce
import numpy as np
import yaml
import os
import pinocchio as pin
import subprocess
import tempfile
import rospkg
from sklearn.preprocessing import StandardScaler
import joblib
import pandas as pd

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

        # First process the xacro file into a URDF
        xacro_path = os.path.join(package_path, 'urdf/panda_arm.urdf.xacro')
        urdf_string = self.process_xacro_to_urdf(xacro_path)
        
        # Create a temporary URDF file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp_urdf:
            tmp_urdf.write(urdf_string)
            tmp_urdf_path = tmp_urdf.name
        
        try:
            # Initialize Pinocchio model from the processed URDF
            self.model_pin = pin.buildModelFromUrdf(tmp_urdf_path)
            self.data_pin = self.model_pin.createData()
            self.frame_id = self.model_pin.getFrameId("ati_net_gamma")
        finally:
            # Clean up the temporary file
            os.unlink(tmp_urdf_path)

        # Initialize panda position relative to origin of the pad (front left)
        self.PANDA_POS = panda_pos

        # Timer for debugging
        self.timer = 0

        # Initialize old q values
        self.previous_q = np.zeros(7)

        # Get and validate calibration type from config file
        self.CALIBRATION_TYPE = rospy.get_param('~calibration_type', 'static') # Default calibration type
        if self.CALIBRATION_TYPE not in self.VALID_CALIBRATION_TYPES:
            rospy.logerr(f"Invalid calibration type: {self.CALIBRATION_TYPE}. Using default 'static'")
            self.CALIBRATION_TYPE = 'static'
        rospy.loginfo(f"Calibration type: {self.CALIBRATION_TYPE}")

        # Initialize calibration parameters for static calibration
        if self.CALIBRATION_TYPE == 'static':
            self.static_calibration_array = []
            self.static_calibration_complete = False
            self.static_calibration_count = 0
            self.static_calibration_offset = np.zeros(7)
            self.STATIC_CALIBRATION_SAMPLES = config['calibration']['static']['num_samples']

        # Load ML model and scalers
        model_dir = os.path.join(package_path, 'models')
        try:
            self.ml_model = joblib.load(os.path.join(model_dir, 'wrench_correction_model.pkl'))
            self.scaler_X = joblib.load(os.path.join(model_dir, 'scaler_X.pkl'))
            self.scaler_y = joblib.load(os.path.join(model_dir, 'scaler_y.pkl'))
            rospy.loginfo("ML model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load ML model: {e}")
            self.ml_model = None
        # Initialize ROS publisher for contact force and wrench data
        self.contact_force_pub = rospy.Publisher('/panda_process_node/contact_force', ContactForce, queue_size=10)
        self.wrench_pub = rospy.Publisher('/panda_process_node/wrench', WrenchStamped, queue_size=10)
        # Create subscriber to the franka_states topic
        self.state_sub = rospy.Subscriber('/franka_state_controller/franka_states',
                                          FrankaState,
                                          self.callback)
        rospy.loginfo("Panda wrapper initialized")

    def process_xacro_to_urdf(self, xacro_path):
        try:
            # Run xacro processing command
            # Note: Adding use_ft_sensor:=true as it appears to be a required argument from your XACRO
            cmd = ['xacro', xacro_path, 'use_ft_sensor:=true']
            result = subprocess.run(cmd, 
                                 capture_output=True, 
                                 text=True, 
                                 check=True)
            return result.stdout
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to process XACRO file: {e.stderr}")

    def normal_jacobian(self, q):
        pin.forwardKinematics(self.model_pin, self.data_pin, q)
        pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)
        J = pin.computeFrameJacobian(self.model_pin, self.data_pin, q, self.frame_id)

        # R_tot = np.zeros((6,6))
        # R_tot[0:3, 0:3] = self.data_pin.oMf[self.frame_id].rotation
        # R_tot[3:, 3:] = self.data_pin.oMf[self.frame_id].rotation
        # J = R_tot @ J

        return J

    def callback(self, msg):
        tau_ext = np.array(msg.tau_ext_hat_filtered)
        q = np.array(msg.q)
        
        q_changed = any(~np.isclose(q, self.previous_q, rtol=1e-5, atol=1e-4))

        # Store current q for next comparison
        self.previous_q = q.copy()
        
        # Process measurement based on calibration type
        if self.CALIBRATION_TYPE == 'static':
            # Redo static calibration if q has changed
            if q_changed:
                self.static_calibration_complete = False
                self.static_calibration_count = 0
                self.static_calibration_array = []
                self.static_calibration_offset = np.zeros(6)
                
            if not self.static_calibration_complete:
                self.static_calibrate(tau_ext)
            else:
                #tau_ext -= self.static_calibration_offset
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
            rospy.loginfo(f"Panda static calibration complete, Offset: {self.static_calibration_offset}")

    def process_data(self, tau_ext, q):
        J = self.normal_jacobian(q)  # Using Pinocchio-based Jacobian calculation
        uncorrected_wrench = self.estimate_external_wrench(J, tau_ext)

        # Apply ML correction if model is loaded
        if self.ml_model is not None:
            try:
                # Create DataFrame with proper feature names
                wrench_data = {
                    'panda_wrench_force_x': uncorrected_wrench[0],   # -0.00064777
                    'panda_wrench_force_y': uncorrected_wrench[1],   # -0.00643131
                    'panda_wrench_force_z': uncorrected_wrench[2],   #  0.07050676
                    'panda_wrench_torque_x': uncorrected_wrench[3],  #  0.03454712
                    'panda_wrench_torque_y': uncorrected_wrench[4],  #  0.00047575
                    'panda_wrench_torque_z': uncorrected_wrench[5]   #  0.00012796
                }
                # Prepare input for ML model
                wrench_input = pd.DataFrame([wrench_data])
                
                # Scale input
                wrench_input_scaled = self.scaler_X.transform(wrench_input)
                
                # Get ML prediction
                corrected_wrench_scaled = self.ml_model.predict(wrench_input_scaled)
                
                # Inverse transform to get actual wrench values
                wrench = self.scaler_y.inverse_transform(corrected_wrench_scaled).flatten()
            except Exception as e:
                rospy.logwarn(f"ML correction failed, using uncorrected wrench: {e}")
                wrench = uncorrected_wrench
        else:
            wrench = uncorrected_wrench

        contact_pos = self.estimate_contact_point(wrench)
        force = wrench[:3]

        self.timer += 1
        if (self.timer >= 500):
            print(f"Tau ext: {tau_ext}")
            print(f"Panda wrench: {wrench}")
            #print(f"Joint positions: {q}")
            # print(f"Panda contact pos {contact_pos}")
            self.timer = 0

        # Create and publish ContactForce message
        contact_msg = ContactForce()
        contact_msg.header.stamp = rospy.Time.now()
        contact_msg.header.frame_id = "pad"
        contact_msg.position.x = contact_pos[0]
        contact_msg.position.y = contact_pos[1]
        contact_msg.position.z = 0.0  # Assume contact point is on the pad
        contact_msg.force.x = force[0]
        contact_msg.force.y = force[1]
        contact_msg.force.z = force[2]
        self.contact_force_pub.publish(contact_msg)

        # Create and publish WrenchStamped message
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = "pad"  # Using same frame as ContactForce
        wrench_msg.wrench.force.x = wrench[0]
        wrench_msg.wrench.force.y = wrench[1]
        wrench_msg.wrench.force.z = wrench[2]
        wrench_msg.wrench.torque.x = wrench[3]
        wrench_msg.wrench.torque.y = wrench[4]
        wrench_msg.wrench.torque.z = wrench[5]
        self.wrench_pub.publish(wrench_msg)

    def dynamic_calibrate(self, measurement):
        rospy.logwarn("Dynamic calibration not yet implemented")

    def estimate_external_wrench(self, J, residuals):
        # Calculate pseudoinverse of Jacobian transpose
        J_pinv = np.linalg.pinv(J.T)

        # Project residuals to get wrench in ft sensor frame (force/moment)
        wrench = J_pinv @ residuals

        T = np.array([
            [1,  0, 0, 0,  0, 0],  # f_x -> f_y
            [0,  0, 1, 0,  0, 0],  # f_y -> f_z
            [0, -1, 0, 0,  0, 0],  # f_z -> f_x
            [0,  0, 0, 1,  0, 0], 
            [0,  0, 0, 0,  0, 1],  
            [0,  0, 0, 0, -1, 0]   
        ])
        wrench = T @ wrench

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

