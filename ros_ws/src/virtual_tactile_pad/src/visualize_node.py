#!/usr/bin/env python
import rospy
from virtual_tactile_pad.msg import ContactForce
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import yaml
import os
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_matrix

# Load configuration
config_path = os.path.join(os.path.dirname(__file__), '../config/config.yaml')
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

class ContactVisualizer:
    def __init__(self):
        """
        Initialize the contact force visualizer.
        Sets up ROS node, publisher for markers, and subscribers for contact force data
        based on parameters.
        """
        # Initialize ROS node
        rospy.init_node('contact_visualizer', anonymous=True)

        # Get parameters
        self.use_ft_sensor = rospy.get_param('~use_ft_sensor', False)
        self.use_panda = rospy.get_param('~use_panda', False)

        # Store latest messages from each source
        self.ft_sensor_msg = None
        self.panda_msg = None

        # Offset for visualization in RViz
        self.X_OFFSET = config['rviz']['position']['x']
        self.Y_OFFSET = config['rviz']['position']['y']
        self.Z_OFFSET = config['rviz']['position']['z']
    
        # Create publisher for visualization markers
        self.marker_pub = rospy.Publisher('visualize_node/contact_force_markers', 
                                        MarkerArray, 
                                        queue_size=10)

        # Set up subscribers based on parameters
        if self.use_ft_sensor:
            rospy.Subscriber('/ft_process_node/contact_force', 
                           ContactForce, 
                           self.ft_sensor_callback)
            rospy.loginfo("Subscribed to FT sensor contact force topic")

        if self.use_panda:
            rospy.Subscriber('/panda_process_node/contact_force',
                           ContactForce,
                           self.panda_callback)
            rospy.loginfo("Subscribed to Panda contact force topic")

        if not (self.use_ft_sensor or self.use_panda):
            rospy.logwarn("No contact force sources enabled. Enable either use_ft_sensor or use_panda parameter.")
        
        # Create a timer to update visualizations periodically
        self.update_timer = rospy.Timer(rospy.Duration(0.1), self.update_visualization)  # 10Hz update rate
        
        rospy.loginfo("Visualizer node initialized")

    def ft_sensor_callback(self, msg):
        self.ft_sensor_msg = msg

    def panda_callback(self, msg):
        self.panda_msg = msg

    def update_visualization(self, event):
        marker_array = MarkerArray()
        marker_id = 0

        # Add FT sensor visualization if available
        if self.ft_sensor_msg is not None:
            if not (self.ft_sensor_msg.position.x == 0 and self.ft_sensor_msg.position.y == 0):
                marker_array.markers.append(
                    self.create_contact_point_marker(self.ft_sensor_msg, marker_id, "ft_sensor", color=(1.0, 0.0, 0.0))
                )
                marker_id += 1
                marker_array.markers.append(
                    self.create_force_arrow_marker(self.ft_sensor_msg, marker_id, "ft_sensor", color=(1.0, 1.0, 0.0))
                )
                marker_id += 1

        # Add Panda visualization if available
        if self.panda_msg is not None:
            if not (self.panda_msg.position.x == 0 and self.panda_msg.position.y == 0):
                marker_array.markers.append(
                    self.create_contact_point_marker(self.panda_msg, marker_id, "panda", color=(0.0, 0.0, 1.0))
                )
                marker_id += 1
                marker_array.markers.append(
                    self.create_force_arrow_marker(self.panda_msg, marker_id, "panda", color=(0.0, 0.5, 1.0))
                )
                marker_id += 1

        # If no valid messages, clear all markers
        if len(marker_array.markers) == 0:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        self.marker_pub.publish(marker_array)

    def create_contact_point_marker(self, msg, marker_id, ns, color):
        contact_marker = Marker()
        contact_marker.header = msg.header
        contact_marker.ns = f"contact_point_{ns}"
        contact_marker.id = marker_id
        contact_marker.type = Marker.SPHERE
        contact_marker.action = Marker.ADD

        # Set position with coordinate transformation
        contact_marker.pose.position.x = msg.position.x + self.X_OFFSET
        contact_marker.pose.position.y = -msg.position.z + self.Z_OFFSET
        contact_marker.pose.position.z = -msg.position.y + self.Y_OFFSET
        contact_marker.pose.orientation.w = 1.0

        # Set marker size (5mm diameter sphere)
        contact_marker.scale.x = 0.005
        contact_marker.scale.y = 0.005
        contact_marker.scale.z = 0.005

        # Set marker color
        contact_marker.color.r = color[0]
        contact_marker.color.g = color[1]
        contact_marker.color.b = color[2]
        contact_marker.color.a = 1.0

        return contact_marker

    def calculate_force_orientation(self, force_vector):
        # Normalize the force vector
        force_norm = np.linalg.norm(force_vector)
        if force_norm < 1e-6:  # Avoid division by zero
            return Quaternion(0, 0, 0, 1)
            
        force_dir = force_vector / force_norm
        
        # The arrow points along its x-axis in RViz, so we need to align it with the force direction
        reference = np.array([1, 0, 0])
        
        # If force direction is parallel to reference, no rotation is needed
        if np.allclose(force_dir, reference) or np.allclose(force_dir, -reference):
            if force_dir[0] < 0:
                # Force points in negative x direction
                return Quaternion(0, 0, 1, 0)  # 180-degree rotation around z-axis
            return Quaternion(0, 0, 0, 1)  # No rotation needed
        
        # Calculate rotation axis and angle
        rotation_axis = np.cross(reference, force_dir)
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        angle = np.arccos(np.dot(reference, force_dir))
        
        # Create rotation matrix
        c = np.cos(angle)
        s = np.sin(angle)
        t = 1 - c
        x, y, z = rotation_axis
        
        rotation_matrix = np.array([
            [t*x*x + c,    t*x*y - z*s,  t*x*z + y*s,  0],
            [t*x*y + z*s,  t*y*y + c,    t*y*z - x*s,  0],
            [t*x*z - y*s,  t*y*z + x*s,  t*z*z + c,    0],
            [0,           0,            0,             1]
        ])
        
        # Convert rotation matrix to quaternion
        q = quaternion_from_matrix(rotation_matrix)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def create_force_arrow_marker(self, msg, marker_id, ns, color):
        force_marker = Marker()
        force_marker.header = msg.header
        force_marker.ns = f"force_arrow_{ns}"
        force_marker.id = marker_id
        force_marker.type = Marker.ARROW
        force_marker.action = Marker.ADD

        # Transform force vector to RViz coordinate system
        force_vector = np.array([
            msg.force.x,
            msg.force.z,  # Transform to RViz coordinate system
            msg.force.y   # Transform to RViz coordinate system
        ])
        
        # Calculate force magnitude and direction
        force_magnitude = np.linalg.norm(force_vector)
        if force_magnitude < 1e-6:
            force_dir = np.array([0, 0, 0])
        else:
            force_dir = force_vector / force_magnitude

        # Calculate arrow length (scaled by force magnitude)
        arrow_length = force_magnitude * 0.01  # Scale factor for visualization

        # Calculate contact position and arrow position
        contact_position = np.array([
            msg.position.x + self.X_OFFSET,
            -msg.position.z + self.Z_OFFSET,
            -msg.position.y + self.Y_OFFSET
        ])
        
        arrow_position = contact_position - force_dir * arrow_length

        # Set arrow position and orientation
        force_marker.pose.position.x = arrow_position[0]
        force_marker.pose.position.y = arrow_position[1]
        force_marker.pose.position.z = arrow_position[2]
        force_marker.pose.orientation = self.calculate_force_orientation(force_vector)

        # Set arrow dimensions
        force_marker.scale.x = arrow_length  # Arrow length
        force_marker.scale.y = 0.005  # Arrow shaft diameter
        force_marker.scale.z = 0.005  # Arrow head diameter

        # Set arrow color
        force_marker.color.r = color[0]
        force_marker.color.g = color[1]
        force_marker.color.b = color[2]
        force_marker.color.a = 1.0

        return force_marker

if __name__ == '__main__':
    try:
        ContactVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass