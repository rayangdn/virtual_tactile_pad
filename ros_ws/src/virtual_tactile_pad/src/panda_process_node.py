#!/usr/bin/env python3

import rospy
from franka_msgs.msg import FrankaState

class FrankaStateSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('franka_state_subscriber', anonymous=True)
        
        # Create subscriber to the franka_states topic
        self.state_sub = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            FrankaState,
            self.state_callback,
            queue_size=1
        )
        
        rospy.loginfo("Franka state subscriber initialized")

    def state_callback(self, msg):
        """Callback function for the franka state messages"""
        # Access different fields of the FrankaState message
        tau_ext = msg.tau_ext_hat_filtered  # Estimated external torque
        joint_torque = msg.tau_J
        joint_positions = msg.q       # Joint positions
        joint_velocities = msg.dq     # Joint velocities
        
        # Example: Print some information
        rospy.loginfo("Received Franka state:")
        rospy.loginfo(f"Joint positions: {joint_positions}")
        rospy.loginfo(f"Tau_ext: {tau_ext}")

    def run(self):
        """Main loop of the subscriber"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down Franka state subscriber")

if __name__ == '__main__':
    try:
        subscriber = FrankaStateSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass