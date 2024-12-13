#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import time

def move_to_joint_positions(move_group, joint_positions):
    """Helper function to move to specific joint positions"""
    print("\nMoving to new joint positions:")
    for name, value in zip(move_group.get_active_joints(), joint_positions):
        print(f"{name}: {value:.4f}")
    
    move_group.set_joint_value_target(joint_positions)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
    if success:
        print("Joint positions reached successfully!")
    else:
        print("Failed to reach joint positions.")
    
    return success

def main():
    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('joint_movement_node', anonymous=True)
    
    # Initialize the robot
    robot = moveit_commander.RobotCommander()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # Wait for the controller
    while not rospy.is_shutdown():
        if move_group.get_current_joint_values():
            break
        rospy.sleep(0.1)
        print("Waiting for controller...")
    
    # Configure the planner
    move_group.set_planning_time(5.0)
    move_group.allow_replanning(True)
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)
    move_group.set_goal_joint_tolerance(0.01)
    move_group.set_planner_id("RRTstar")
    
    # Define the three joint configurations
    pose1 = [-1.5028, -0.0291, 1.4637, -1.9956, 0.2882, 2.9601, -0.2818]
    pose2 = [-1.2255, -0.6390, 1.7476, -2.1473, 0.8202, 2.9605, 0.0039]
    pose3 = [-1.4230, 0.7972, 1.0168, -1.8400, -0.4719, 2.9639, -0.3244]
    
    try:
        print("\nStarting infinite loop movement...")
        while not rospy.is_shutdown():
            # Move to first pose
            if not move_to_joint_positions(move_group, pose1):
                print("Failed to reach pose 1. Continuing...")
            
            # Move to second pose
            if not move_to_joint_positions(move_group, pose2):
                print("Failed to reach pose 2. Continuing...")
            
            # Move to third pose
            if not move_to_joint_positions(move_group, pose3):
                print("Failed to reach pose 3. Continuing...")
                
            print("\nCompleted one loop, starting next iteration...")
            
    except KeyboardInterrupt:
        print("\nStopping the robot...")
        move_group.stop()
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python3
# import sys
# import rospy
# import moveit_commander
# from moveit_msgs.msg import RobotTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# import time

# def move_to_start_position(move_group, joint_positions):
#     """Move to the initial position before starting the trajectory"""
#     print("\nMoving to initial position...")
#     for name, value in zip(move_group.get_active_joints(), joint_positions):
#         print(f"{name}: {value:.4f}")
    
#     move_group.set_joint_value_target(joint_positions)
#     success = move_group.go(wait=True)
#     move_group.stop()
#     move_group.clear_pose_targets()
    
#     if success:
#         print("Initial position reached successfully!")
#     else:
#         print("Failed to reach initial position.")
    
#     return success

# def create_trajectory(move_group, waypoints):
#     """Create a smooth trajectory through the given joint positions"""
#     trajectory = RobotTrajectory()
#     trajectory.joint_trajectory.joint_names = move_group.get_active_joints()

#     # Time parameterization for smooth movement
#     time_interval = 6.0  # seconds between waypoints
#     for i, joint_positions in enumerate(waypoints):
#         point = JointTrajectoryPoint()
#         point.positions = joint_positions
#         point.time_from_start = rospy.Duration(time_interval * i)
#         trajectory.joint_trajectory.points.append(point)

#     return trajectory

# def plan_path(move_group, waypoints):
#     """Plan a smooth path through all waypoints"""
#     print("\nPlanning path through all waypoints...")
    
#     # Create initial trajectory
#     trajectory = create_trajectory(move_group, waypoints)
    
#     # Use time parameterization to create a smooth trajectory
#     move_group.set_max_velocity_scaling_factor(0.1)  # 20% speed
#     move_group.set_max_acceleration_scaling_factor(0.1)
    
#     # Compute trajectory (this will smooth out the path)
#     success = move_group.execute(trajectory, wait=True)
    
#     if success:
#         print("Path planning and execution successful!")
#     else:
#         print("Failed to plan/execute path.")
    
#     return success, trajectory

# def main():
#     # Initialize moveit_commander and rospy node
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('smooth_joint_movement_node', anonymous=True)
    
#     # Initialize the robot
#     robot = moveit_commander.RobotCommander()
#     group_name = "panda_arm"
#     move_group = moveit_commander.MoveGroupCommander(group_name)
    
#     # Wait for the controller
#     while not rospy.is_shutdown():
#         if move_group.get_current_joint_values():
#             break
#         rospy.sleep(0.1)
#         print("Waiting for controller...")
    
#     # Configure the planner
#     move_group.set_planning_time(5.0)
#     move_group.allow_replanning(True)
#     move_group.set_goal_joint_tolerance(0.01)
#     move_group.set_planner_id("RRTstar")
    
#     # Define initial position (Pose 1)
#     initial_position = [-1.5028, -0.0291, 1.4637, -1.9956, 0.2882, 2.9601, -0.2818]
    
#     # Define the waypoints for the continuous trajectory
#     waypoints = [
#         [-1.5028, -0.0291, 1.4637, -1.9956, 0.2882, 2.9601, -0.2818],   # Pose 1
#         [-1.2255, -0.6390, 1.7476, -2.1473, 0.8202, 2.9605, 0.0039],   # Pose 2
#         [-1.4230, 0.7972, 1.0168, -1.8400, -0.4719, 2.9639, -0.3244],  # Pose 3
#         [-1.5028, -0.0291, 1.4637, -1.9956, 0.2882, 2.9601, -0.2818]   # Back to Pose 1
#     ]
    
#     try:
#         # First move to the initial position
#         print("\nMoving to initial position...")
#         move_group.set_max_velocity_scaling_factor(0.1)  # Slower for initial positioning
#         move_group.set_max_acceleration_scaling_factor(0.1)
        
#         if not move_to_start_position(move_group, initial_position):
#             print("Failed to reach initial position. Exiting...")
#             moveit_commander.roscpp_shutdown()
#             return
            
#         print("\nWaiting 2 seconds before starting trajectory...")
#         rospy.sleep(2.0)
        
#         # Now plan and execute the continuous trajectory
#         print("\nStarting continuous trajectory...")
#         success, planned_trajectory = plan_path(move_group, waypoints)
        
#         if success:
#             print("\nPath executed successfully! Press Ctrl+C to stop...")
            
#             # Continue executing the same trajectory
#             while not rospy.is_shutdown():
#                 move_group.execute(planned_trajectory, wait=True)
#                 print("\nCompleted one loop, starting next iteration...")
#                 break
                
#     except KeyboardInterrupt:
#         print("\nStopping the robot...")
#         move_group.stop()
#         moveit_commander.roscpp_shutdown()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass