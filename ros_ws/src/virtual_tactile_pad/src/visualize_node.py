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
        Sets up ROS node, publisher for markers, and subscriber for contact force data.
        """
        # Initialize ROS node
        rospy.init_node('contact_visualizer', anonymous=True)

        # Offset for visualization in RViz (to align with tactile pad frame)
        self.X_OFFSET = config['rviz']['position']['x']
        self.y_OFFSET = config['rviz']['position']['y']
        self.Z_OFFSET = config['rviz']['position']['z']
    
        # Create publisher for visualization markers
        self.marker_pub = rospy.Publisher('visualize_node/contact_force_markers', 
                                        MarkerArray, 
                                        queue_size=10)

        # Subscribe to contact force messages from the FT sensor
        rospy.Subscriber('/ft_process_node/contact_force', 
                        ContactForce, 
                        self.contact_force_callback)
        
        rospy.loginfo("Visualizer node started")

    def contact_force_callback(self, msg):
       # Skip visualization if position is at origin
        if msg.position.x == 0 and msg.position.y == 0:
            # Publish empty marker array to clear previous markers
             # Create a marker array with a deletion marker
            marker_array = MarkerArray()
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL  # This will clear all markers
            delete_marker.ns = "your_namespace"  # Use the same namespace as your arrows
            marker_array.markers.append(delete_marker)
            self.marker_pub.publish(marker_array)
            return
        marker_array = MarkerArray()
        marker_array.markers.append(self.create_contact_point_marker(msg))
        marker_array.markers.append(self.create_force_arrow_marker(msg))
        self.marker_pub.publish(marker_array)

    def create_contact_point_marker(self, msg):
        contact_marker = Marker()
        contact_marker.header = msg.header
        contact_marker.ns = "contact_point"
        contact_marker.id = 0
        contact_marker.type = Marker.SPHERE
        contact_marker.action = Marker.ADD

        # Set position with coordinate transformation
        contact_marker.pose.position.x = msg.position.x + self.X_OFFSET
        contact_marker.pose.position.y = -msg.position.z + self.Z_OFFSET
        contact_marker.pose.position.z = -msg.position.y + self.y_OFFSET
        contact_marker.pose.orientation.w = 1.0

        # Set marker size (5mm diameter sphere)
        contact_marker.scale.x = 0.005
        contact_marker.scale.y = 0.005
        contact_marker.scale.z = 0.005

        # Set marker color (red)
        contact_marker.color.r = 1.0
        contact_marker.color.g = 0.0
        contact_marker.color.b = 0.0
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

    def create_force_arrow_marker(self, msg):
        force_marker = Marker()
        force_marker.header = msg.header
        force_marker.ns = "force_arrow"
        force_marker.id = 1
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

        # Shift arrow position backward from contact point
        # The position needs to be shifted by arrow_length in the opposite direction of the force
        contact_position = np.array([
            msg.position.x + self.X_OFFSET,
            -msg.position.z + self.Z_OFFSET,
            -msg.position.y + self.y_OFFSET
        ])
        
        # Calculate shifted position (arrow start point)
        arrow_position = contact_position - force_dir * arrow_length

        # Set arrow position
        force_marker.pose.position.x = arrow_position[0]
        force_marker.pose.position.y = arrow_position[1]
        force_marker.pose.position.z = arrow_position[2]

        # Calculate and set arrow orientation based on force direction
        orientation = self.calculate_force_orientation(force_vector)
        force_marker.pose.orientation = orientation

        # Set arrow dimensions
        force_marker.scale.x = arrow_length  # Arrow length
        force_marker.scale.y = 0.005  # Arrow shaft diameter
        force_marker.scale.z = 0.005  # Arrow head diameter

        # Set arrow color (yellow)
        force_marker.color.r = 1.0
        force_marker.color.g = 1.0
        force_marker.color.b = 0.0
        force_marker.color.a = 1.0

        return force_marker

if __name__ == '__main__':
    try:
        ContactVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass