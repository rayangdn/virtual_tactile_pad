#!/usr/bin/env python3

import rospy
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import WrenchStamped
from virtual_tactile_pad.msg import ContactForce
from std_msgs.msg import Float64MultiArray
import numpy as np
import yaml
import os
import pinocchio as pin
import subprocess
import tempfile
import rospkg
import torch
import torch.nn as nn

# Load configuration
config_path = os.path.join(os.path.dirname(__file__), '../config/config.yaml')
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

# Get package path using rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path('virtual_tactile_pad')


class PandaWrapper:

    VALID_CALIBRATION_TYPES = config['calibration']['types']

    def __init__(self, panda_pos=np.array([config['panda']['position']['x'],
                                           config['panda']['position']['y'],
                                           config['panda']['position']['z']], dtype=float)):

        # Initialize ROS node
        rospy.init_node('panda_process', anonymous=True)

        # Initialize particle filter parameters from config
        self.num_particles = config['particle_filter']['num_particles']
        
        # Get noise parameters for x and y from config
        self.process_noise_std = {
            'x': config['particle_filter']['process_noise_std']['x'],
            'y': config['particle_filter']['process_noise_std']['y']
        }
        
        self.measurement_noise_std = {
            'x': config['particle_filter']['measurement_noise_std']['x'],
            'y': config['particle_filter']['measurement_noise_std']['y']
        }
        # Get pad dimensions
        self.pad_dims = [
            config['pad']['dimensions']['x'],
            config['pad']['dimensions']['y']
        ]
        
        # Initialize particles (x, y positions)
        self.particles = np.random.uniform(
            low=[0.0, 0.0],
            high=self.pad_dims,
            size=(self.num_particles, 2)
        )
        self.weights = np.ones(self.num_particles) / self.num_particles

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

        # Get and validate calibration type from config file
        self.CALIBRATION_TYPE = rospy.get_param('~calibration_type', 'static')
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

        # Initialize panda ROS publisher 
        self.contact_force_pub = rospy.Publisher('/panda_process_node/contact_force', ContactForce, queue_size=10)
        self.wrench_pub = rospy.Publisher('/panda_process_node/wrench', WrenchStamped, queue_size=10)
        self.tau_ext_pub = rospy.Publisher('/panda_process_node/tau_ext', Float64MultiArray, queue_size=10)
        self.raw_contact_pos = rospy.Publisher('/panda_process_node/raw_contact_pos', Float64MultiArray, queue_size=10)
        self.jacobian_pub = rospy.Publisher('/panda_process_node/jacobian', Float64MultiArray, queue_size=10)

        # Create subscriber to the franka_states topic
        self.state_sub = rospy.Subscriber('/franka_state_controller/franka_states',
                                          FrankaState,
                                          self.callback)
        rospy.loginfo("Panda wrapper initialized")

    def process_xacro_to_urdf(self, xacro_path):
        try:
            # Run xacro processing command
            # Note: Adding use_ft_sensor:=true as it is a required argument
            cmd = ['xacro', xacro_path, 'use_ft_sensor:=true']
            result = subprocess.run(cmd, 
                                 capture_output=True, 
                                 text=True, 
                                 check=True)
            return result.stdout
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to process XACRO file: {e.stderr}")

    def gaussian_prob(self, x, mu, sigma):
        return (1.0 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mu) / sigma) ** 2)

    def predict_particles(self):
        # Apply different noise levels for x and y
        noise_x = np.random.normal(0, self.process_noise_std['x'], size=self.num_particles)
        noise_y = np.random.normal(0, self.process_noise_std['y'], size=self.num_particles)
        
        self.particles[:, 0] += noise_x
        self.particles[:, 1] += noise_y

        # Ensure particles stay within pad boundaries
        self.particles = np.clip(
            self.particles,
            [0.0, 0.0],
            self.pad_dims
        )

    def update_particle_weights(self, measurement_pos):
        # Calculate likelihoods with different noise parameters for x and y
        likelihood_x = self.gaussian_prob(
            self.particles[:, 0],
            measurement_pos[0],
            self.measurement_noise_std['x']
        )
        likelihood_y = self.gaussian_prob(
            self.particles[:, 1],
            measurement_pos[1],
            self.measurement_noise_std['y']
        )
        
        # Update weights
        self.weights *= likelihood_x * likelihood_y
        
        # Normalize weights
        self.weights += 1e-300  # Avoid division by zero
        self.weights /= np.sum(self.weights)

    def resample_particles(self):
        # Systematic resampling
        cumsum = np.cumsum(self.weights)
        cumsum[-1] = 1.0  # Handle numerical errors
        
        # Generate points for resampling
        positions = (np.random.random() + np.arange(self.num_particles)) / self.num_particles
        
        # Resample particles
        indexes = np.searchsorted(cumsum, positions)
        self.particles = self.particles[indexes]
        
        # Reset weights
        self.weights = np.ones(self.num_particles) / self.num_particles

    def get_filtered_position(self):
        return np.average(self.particles, weights=self.weights, axis=0)

    def normal_jacobian(self, q):
        pin.forwardKinematics(self.model_pin, self.data_pin, q)
        pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)
        J = pin.computeFrameJacobian(self.model_pin, self.data_pin, q, self.frame_id)

        # R_tot = np.zeros((6,6))
        # R_tot[0:3, 0:3] = self.data_pin.oMf[self.frame_id].rotation
        # R_tot[3:, 3:] = self.data_pin.oMf[self.frame_id].rotation
        # J = R_tot @ J

        return J

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

    def static_calibrate(self, tau_ext):
        self.static_calibration_array.append(tau_ext)
        self.static_calibration_count += 1

        if self.static_calibration_count >= self.STATIC_CALIBRATION_SAMPLES:
            self.static_calibration_offset = np.mean(self.static_calibration_array, axis=0)
            self.static_calibration_complete = True
            rospy.loginfo(f"Panda static calibration complete. Offset: {self.static_calibration_offset}")

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
            self.process_data(tau_ext, q)
        
    def process_data(self, tau_ext, q):

        J = self.normal_jacobian(q)
        wrench = self.estimate_external_wrench(J, tau_ext)

        # Use corrected wrench for contact point estimation
        contact_pos = self.estimate_contact_point(wrench)
        raw_contact_pos = contact_pos.copy()
        force = wrench[:3]
        # Apply particle filter if force magnitude is above threshold
        force_magnitude = np.linalg.norm(force)
        if force_magnitude > config['processing']['force_threshold']:
            # Particle filter steps
            self.predict_particles()
            self.update_particle_weights(contact_pos[:2])  # Only use x, y coordinates
            
            # Resample if effective sample size is too low
            n_eff = 1.0 / np.sum(self.weights ** 2)
            if n_eff < self.num_particles / 2:
                self.resample_particles()
            
            # Get filtered position
            filtered_pos = self.get_filtered_position()
            
            # Update contact position with filtered x, y coordinates
            contact_pos[:2] = filtered_pos

        self.timer += 1
        if (self.timer >= 500):
            # print(f"Tau externe: {tau_ext}")
            # print(f"Original wrench: {wrench}")
            # print(f"Panda wrench: {wrench}")
            self.timer = 0

        # Publish messages with corrected wrench
        self.publish_messages(wrench, raw_contact_pos, contact_pos, force, tau_ext, J)

    def publish_messages(self, wrench, raw_contact_pos, contact_pos, force, tau_ext, jacobian):
        # Contact Force message
        contact_msg = ContactForce()
        contact_msg.header.stamp = rospy.Time.now()
        contact_msg.header.frame_id = "pad"
        contact_msg.position.x = contact_pos[0]
        contact_msg.position.y = contact_pos[1]
        contact_msg.position.z = 0.0
        contact_msg.force.x = force[0]
        contact_msg.force.y = force[1]
        contact_msg.force.z = force[2]
        self.contact_force_pub.publish(contact_msg)

        # Wrench message
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = "pad"
        wrench_msg.wrench.force.x = wrench[0]
        wrench_msg.wrench.force.y = wrench[1]
        wrench_msg.wrench.force.z = wrench[2]
        wrench_msg.wrench.torque.x = wrench[3]
        wrench_msg.wrench.torque.y = wrench[4]
        wrench_msg.wrench.torque.z = wrench[5]
        self.wrench_pub.publish(wrench_msg)

        # Other messages

        raw_contact_pos_msg = Float64MultiArray()
        raw_contact_pos_msg.data = raw_contact_pos.tolist()  
        self.raw_contact_pos.publish(raw_contact_pos_msg)

 
        tau_msg = Float64MultiArray()
        tau_msg.data = tau_ext.tolist()
        self.tau_ext_pub.publish(tau_msg)

        jacobian_msg = Float64MultiArray()
        jacobian_msg.data = jacobian.flatten().tolist()
        self.jacobian_pub.publish(jacobian_msg)

if __name__ == '__main__':
    try:
        PandaWrapper()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rospy.signal_shutdown("User interrupted")

