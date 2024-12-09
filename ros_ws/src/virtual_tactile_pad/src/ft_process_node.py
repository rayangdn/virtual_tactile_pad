#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped
from virtual_tactile_pad.msg import ContactForce
import numpy as np
import yaml
import os

# Load configuration
config_path = os.path.join(os.path.dirname(__file__), '../config/config.yaml')
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

class FTSensorSimulator:
    def __init__(self, position):
        self.t = 0  # Time variable for simulation
        self.NOISE = config['sensor']['simulation']['noise_magnitude']  # Magnitude of noise to add to measurements
        self.position = position

    def generate_data(self):
        # Generate force based on simulation time
        if self.t <= 1.0:
            force = np.array([0, 0, 0])
        elif self.t <= 10.0:
            force = np.array([0.0, -3.0, -3.0])
        elif self.t <= 12.0:
            force = np.array([0, 0, 0])
        elif self.t <= 21.0:
            force = np.array([0, 0, -5.0])
        else:
            force = np.array([0, 0, 0])
        
        application_point = np.array([
            0.06 + 0.03 * np.sin(2.0 * self.t),
            0.07 + 0.04 * np.cos(1.0 * self.t),
            0.0
        ])
        
        # Calculate moment arm and resulting torque
        r = application_point - self.position
        torque = np.cross(r, force)
        
        # Combine force and torque measurements
        measurement = np.concatenate([force, torque])
        
        # Add noise proportional to measurement magnitude
        noise = np.random.normal(0, self.NOISE, 6)
        measurement = measurement + noise * abs(measurement)
        self.t += 0.1
        return measurement

class FTSensorWrapper:
    
    VALID_CALIBRATION_TYPES = config['calibration']['types']
    
    def __init__(self, sensor_pos=np.array([config['sensor']['position']['x'],
                                            config['sensor']['position']['y'],
                                            config['sensor']['position']['z']], dtype=float)):
        # Initialize ROS node
        rospy.init_node('ft_process', anonymous=True)

        # Initialize sensor position relative to origin of the pad (front left)
        self.SENSOR_POS = sensor_pos

        # Timer for debugging
        self.timer = 0

        # Get and validate simulator parameter from config file
        self.USE_SIMULATOR = rospy.get_param('~use_simulation', False)  # '~' means private parameter
        if self.USE_SIMULATOR:
            rospy.loginfo("Mode: Simulation")
            self.simulator = FTSensorSimulator(self.SENSOR_POS)
        else:
            rospy.loginfo("Mode: Real Sensor")

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
            if (self.USE_SIMULATOR):
                self.STATIC_CALIBRATION_SAMPLES = 10 # Use fewer samples for simulation
        
        # Initialize ROS publisher for contact force and wrench data
        self.contact_force_pub = rospy.Publisher('/ft_process_node/contact_force', ContactForce, queue_size=10)
        self.wrench_pub = rospy.Publisher('/ft_process_node/wrench', WrenchStamped, queue_size=10)

    def start(self):
        if self.USE_SIMULATOR:
            rospy.Timer(rospy.Duration(0.1), self.simulated_callback)
            rospy.loginfo("FT sensor simulator started")
        else:
            self.ft_sub = rospy.Subscriber("/ft_sensor/netft_data", 
                                           WrenchStamped, 
                                           self.callback)
            rospy.loginfo("FT sensor wrapper started")

    def callback(self, data):
        wrench = np.array([
            data.wrench.force.x,
            data.wrench.force.z, # Swap y and z axes
            data.wrench.force.y,
            data.wrench.torque.x,
            data.wrench.torque.z,
            data.wrench.torque.y
        ])
        
        # Process measurement based on calibration type
        if self.CALIBRATION_TYPE == 'static':
            if not self.static_calibration_complete:
                self.static_calibrate(wrench)
            else:
                self.process_data(wrench - self.static_calibration_offset)
        else:
            self.dynamic_calibrate(wrench)
            self.process_data(wrench)

    def simulated_callback(self, event):
        wrench = self.simulator.generate_data()
        if self.CALIBRATION_TYPE == 'static':
            if not self.static_calibration_complete:
                self.static_calibrate(wrench)
            else:
                self.process_data(wrench - self.static_calibration_offset)
        else:
            self.dynamic_calibrate(wrench)
            self.process_data(wrench)

    def static_calibrate(self, wrench):
        self.static_calibration_array.append(wrench)
        self.static_calibration_count += 1

        if self.static_calibration_count >= self.STATIC_CALIBRATION_SAMPLES:
            self.static_calibration_offset = np.mean(self.static_calibration_array, axis=0)
            self.static_calibration_complete = True
            rospy.loginfo(f"FT static calibration complete. Offset: {self.static_calibration_offset}")

    def dynamic_calibrate(self, wrench):
        rospy.logwarn("Dynamic calibration not yet implemented")
        
    def process_data(self, wrench):
        # Calculate contact point from force/torque measurements
        contact_pos = self.estimate_contact_point(wrench)
        force = wrench[:3]

        self.timer += 1
        if (self.timer >= 500):
            # print(f"FT wrench: {wrench}")
            # print(f"FT contact pos {contact_pos}")
            # print(f"q: {q}")
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

    def estimate_contact_point(self, wrench):
        force = wrench[:3]
        moment = wrench[3:]

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
        b = np.concatenate([moment, [-self.SENSOR_POS[2]]])

        p_c_d = np.linalg.lstsq(A, b, rcond=None)[0]
        estimated_point = p_c_d + self.SENSOR_POS # Add sensor position to get contact point in pad frame
        if self.USE_SIMULATOR==False:
            estimated_point[0] = config['pad']['dimensions']['x'] - estimated_point[0] # Convert to pad frame
            
        return estimated_point

if __name__ == '__main__':
    try:
        ft_sensor = FTSensorWrapper()
        ft_sensor.start()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rospy.signal_shutdown("User interrupted")