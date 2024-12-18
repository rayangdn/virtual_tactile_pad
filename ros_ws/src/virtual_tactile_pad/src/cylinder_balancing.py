#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from virtual_tactile_pad.msg import ContactForce
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from enum import Enum
import numpy as np

# Define states using Enum for better code organization
class RobotState(Enum):
    IDLE = 1
    HOMING = 2
    BALANCING = 3

class CylinderBalancing:
    def __init__(self):
        rospy.init_node('cylinder_balancing', anonymous=True)
        
        # State machine initialization
        self.current_state = RobotState.IDLE
        self.homing_completed = False

        # Control parameters
        self.K_p = -0.15  # Proportional gain - tune this value
        self.max_error = 0.06  # Maximum error magnitude allowed
        self.goal_position = 0.06  # Target x position in meters
        
        # Joint limits for Franka Emika Panda
        self.joint_limits = {
            'lower': [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
            'upper': [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
            'velocity': [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0]  # Added conservative velocity limits (rad/s)
        }
        
        self.ready_position = [0.0825, 0.9376, -0.0648, -0.7659, -0.0050, 3.2559, 0.0357]
        
        # Joint names for the trajectory message
        self.joint_names = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7'
        ]
        
        self.joint_states_sub = rospy.Subscriber(
            "/franka_state_controller/joint_states",
            JointState,
            self.joint_states_callback
        )
        
        self.trajectory_pub = rospy.Publisher(
            '/position_joint_trajectory_controller/command',
            JointTrajectory,
            queue_size=10
        )

        self.contact_force_sub = rospy.Subscriber(
            '/ft_process_node/contact_force',
            ContactForce,
            self.contact_force_callback
        )
        
        self.contact_pos = None
        self.joint_positions = None
        
        # Timer for state machine update
        self.update_rate = 100  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.state_machine_update)
        
        rospy.loginfo("Cylinder Balancing node initialized")
        
    def check_joint_limits(self, positions):
        for i, pos in enumerate(positions):
            if pos < self.joint_limits['lower'][i] or pos > self.joint_limits['upper'][i]:
                rospy.logerr(f"Joint {i+1} position {pos} exceeds limits!")
                return False
        return True
    
    def contact_force_callback(self, msg):
        self.contact_pos = [msg.position.x, msg.position.y]
    
    def joint_states_callback(self, msg):
        self.joint_positions = msg.position
        
    def move_joints_slow(self, target_positions, max_velocity=0.15):
        if len(target_positions) != 7:
            rospy.logerr("Expected 7 joint positions!")
            return None
            
        if not self.check_joint_limits(target_positions):
            rospy.logerr("Movement aborted due to joint limits!")
            return None
            
        while self.joint_positions is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        max_movement = 0
        for i in range(7):
            movement = abs(target_positions[i] - self.joint_positions[i])
            max_movement = max(max_movement, movement)
        
        actual_duration = max_movement / max_velocity
        
        rospy.loginfo(f"Moving to target over {actual_duration:.2f} seconds")
        
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * 7
        point.accelerations = [0.0] * 7
        point.time_from_start = rospy.Duration(actual_duration)
        
        traj.points = [point]
        
        self.trajectory_pub.publish(traj)
        return actual_duration
    
    def send_joint_command(self, last_joint_position):
        if self.joint_positions is None:
            return
            
        # Create trajectory for all joints
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        # Keep current positions for first 6 joints, update last joint
        point.positions = list(self.ready_position[:-1]) + [last_joint_position]
        point.velocities = [0.0] * 7
        point.accelerations = [0.0] * 7
        point.time_from_start = rospy.Duration(0.001)  # Very short duration for reactivity
        
        traj.points = [point]
        self.trajectory_pub.publish(traj)
        self.last_command_time = rospy.Time.now()
    
    def calculate_control(self):
        if (self.contact_pos is None) or (self.contact_pos == [0.0, 0.0]) or (self.joint_positions is None):
            return None
            
        # Calculate error (desired - current)
        error = self.goal_position - self.contact_pos[0]

        # Clip error to ±0.06
        error = np.clip(error, -self.max_error, self.max_error)
        
        # Calculate control output using proportional control
        control = self.K_p * error
        
        # Get current position of last joint
        current_pos = self.joint_positions[-1]
        
        # Calculate new position command
        new_pos = current_pos + control
        
        # Limit the position within joint limits
        new_pos = np.clip(new_pos, 
                         self.joint_limits['lower'][-1],
                         self.joint_limits['upper'][-1])

        return new_pos
        
    def move_to_ready(self, max_velocity=0.1):
        rospy.loginfo("Moving to ready position...")
        return self.move_joints_slow(self.ready_position, max_velocity=max_velocity)

    def state_machine_update(self, event):
        if self.current_state == RobotState.IDLE:
            # Do nothing in IDLE state
            pass
            
        elif self.current_state == RobotState.HOMING:
            if not self.homing_completed:
                duration = self.move_to_ready(max_velocity=0.1)
                if duration is not None:
                    rospy.sleep(duration)
                    self.homing_completed = True
                    rospy.loginfo("Homing completed")
                    # Automatically transition to BALANCING state
                    self.set_state(RobotState.BALANCING)
                
        elif self.current_state == RobotState.BALANCING:
                
            new_pos = self.calculate_control()
            if new_pos is not None:
                self.send_joint_command(new_pos)

    def set_state(self, new_state):
        """External interface to change states"""
        if new_state != self.current_state:
            rospy.loginfo(f"Changing state from {self.current_state} to {new_state}")
            self.current_state = new_state
            # Reset flags when changing states
            if new_state == RobotState.HOMING:
                self.homing_completed = False

def main():
    try:
        cylinder_eq = CylinderBalancing()
        
        # Start in IDLE state by default
        rospy.loginfo("Starting in IDLE state")
        rospy.sleep(1)
        
        # Transition to HOMING
        cylinder_eq.set_state(RobotState.HOMING)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()