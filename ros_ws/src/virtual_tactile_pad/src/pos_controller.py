#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg

def main():
    # Initialize moveit commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_robot_node', anonymous=True)

    # Instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a MoveGroupCommander object
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set planning time and allow replanning
    move_group.set_planning_time(10)
    move_group.allow_replanning(True)

    # Optional: Set max velocity and acceleration scaling
    move_group.set_max_velocity_scaling_factor(0.5)
    move_group.set_max_acceleration_scaling_factor(0.5)

    # Move to a named configuration (if defined in SRDF)
    print("Moving to ready pose...")
    move_group.go("ready", wait=True)
    move_group.stop()

    # Move to specific joint positions
    print("Moving to specific joint positions...")
    joint_goal = [0, -0.785, 0, -2.356, 0, 1.57, 0.785]
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # Move to a specific Cartesian pose
    print("Moving to specific Cartesian pose...")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)
    
    # Plan and execute the movement
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Shutdown
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass