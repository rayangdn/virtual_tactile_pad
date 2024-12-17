#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import time
import signal
from geometry_msgs.msg import Pose, Point, Quaternion
import math

class CartesianMovement:
    def __init__(self):
        # Initialize moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('cartesian_movement', anonymous=True)

        # Initialize shutdown flag
        self.shutdown_flag = False

        # Register shutdown handlers
        rospy.on_shutdown(self.shutdown_handler)
        signal.signal(signal.SIGINT, self.signal_handler)

        # Initialize the robot
        self.robot = moveit_commander.RobotCommander()
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Configure the planner
        self.move_group.set_planning_time(5.0)
        self.move_group.allow_replanning(True)

        # self.move_group.set_max_velocity_scaling_factor(0.01)
        # self.move_group.set_max_acceleration_scaling_factor(0.01) # LOW

        self.move_group.set_max_velocity_scaling_factor(0.05)
        self.move_group.set_max_acceleration_scaling_factor(0.05) # MID

        # self.move_group.set_max_velocity_scaling_factor(0.1)
        # self.move_group.set_max_acceleration_scaling_factor(0.1) # HIGH


        self.move_group.set_goal_position_tolerance(0.01)  # 1mm tolerance
        self.move_group.set_goal_orientation_tolerance(0.01)  # ~0.57 degrees
        self.move_group.set_planner_id("RRTstar")

        self.start_joints = [-0.8001161583157639, 0.30887033649841145, 1.0422925187635175, 
                             -1.8155412556512804, -0.24074681758218341, 2.5782509957748516, 
                              0.15934831999525723]


        if not self.move_to_joint_positions(self.start_joints):
            print("Failed to reach start position. Continuing...")

        # TRANSLATION MOVEMENT
        # Define the two poses

        # Get current pose as starting point
        current_pose = self.move_group.get_current_pose().pose
        
        # Create two poses with different heights
        self.pose1 = Pose()
        self.pose1.position = Point(
            x=current_pose.position.x - 0.1,  # Move 20cm back from current position
            y=current_pose.position.y - 0.2, # Move 30cm left from current position
            z=current_pose.position.z + 0.1  # Move 40cm up from current position
        )
        self.pose1.orientation = current_pose.orientation

        self.pose2 = Pose()
        self.pose2.position = Point(
            x=current_pose.position.x - 0.1,
            y=current_pose.position.y + 0.2,
            z=current_pose.position.z + 0.1
        )
        self.pose2.orientation = current_pose.orientation

        self.pose3 = Pose()
        self.pose3.position = Point(
            x=current_pose.position.x,
            y=current_pose.position.y,
            z=current_pose.position.z
        )
        self.pose3.orientation = current_pose.orientation

        # ORIENTATION MOVEMENT
        # # Get current joint values
        #self.current_joints = self.move_group.get_current_joint_values()

        # # Create two joint configurations for rotation
        # self.joints1 = list(self.current_joints)
        # self.joints2 = list(self.current_joints)
        
        # rotation_angle_1 = math.radians(45)
        # rotation_angle_2 = math.radians(30)

        # Rotate along y axis 
        # self.joints1[-1] = self.current_joints[-1] + rotation_angle_1
        # self.joints2[-1] = self.current_joints[-1] - rotation_angle_1

        # # Rotate along x axis 
        # self.joints1[-2] = self.current_joints[-2] + rotation_angle_2
        # self.joints2[-2] = self.current_joints[-2] - rotation_angle_2

    def move_to_joint_positions(self, joint_positions):
        if self.shutdown_flag:
            return False

        print("\nMoving to new joint positions:")
        print(f"Last joint angle: {math.degrees(joint_positions[-1]):.2f} degrees")

        self.move_group.set_joint_value_target(joint_positions)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if success:
            print("Joint positions reached successfully!")
        else:
            print("Failed to reach joint positions.")

        return success

    def move_to_pose(self, target_pose):
        if self.shutdown_flag:
            return False

        print("\nMoving to new pose:")
        print(f"Position: x={target_pose.position.x:.4f}, y={target_pose.position.y:.4f}, z={target_pose.position.z:.4f}")
        print(f"Orientation: x={target_pose.orientation.x:.4f}, y={target_pose.orientation.y:.4f}, "
              f"z={target_pose.orientation.z:.4f}, w={target_pose.orientation.w:.4f}")

        self.move_group.set_pose_target(target_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if success:
            print("Pose reached successfully!")
        else:
            print("Failed to reach pose.")

        return success

    def shutdown_handler(self):
        print("\nShutdown requested...")
        self.shutdown_flag = True
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()

    def signal_handler(self, signum, frame):
        self.shutdown_handler()

    def run(self):
        # Wait for the controller
        while not rospy.is_shutdown() and not self.shutdown_flag:
            if self.move_group.get_current_joint_values():
                break
            rospy.sleep(0.1)
            print("Waiting for controller...")


        print("\nStarting movement loop...")

        if not self.move_to_joint_positions(self.start_joints):
            print("Failed to reach start position. Continuing...")

        while not rospy.is_shutdown() and not self.shutdown_flag:
            try:
                # TRANSLATION MOVEMENT
                # Move to position 1
                if not self.move_to_pose(self.pose1):
                    print("Failed to reach position 1. Continuing...")
                    if self.shutdown_flag:
                        break

                # Move to position 2
                if not self.move_to_pose(self.pose2):
                    print("Failed to reach position 2. Continuing...")
                    if self.shutdown_flag:
                        break

                # Move to position 3
                if not self.move_to_pose(self.pose3):
                    print("Failed to reach position 3. Continuing...")
                    if self.shutdown_flag:
                        break

                # ROTATION MOVEMENT
                # Rotate joints 1
                # if not self.move_to_joint_positions(self.joints1):
                #     print("Failed to reach rotation 1. Continuing...")
                #     if self.shutdown_flag:
                #         break

                # # Rotate joints 2
                # if not self.move_to_joint_positions(self.joints2):
                #     print("Failed to reach rotation 2. Continuing...")
                #     if self.shutdown_flag:
                #         break

                print("\nCompleted one movement cycle, starting next iteration...")

            except Exception as e:
                print(f"Error during movement: {e}")
                self.shutdown_handler()
                break

def main():
    try:
        movement = CartesianMovement()
        movement.run()
    except Exception as e:
        print(f"Error initializing: {e}")
    finally:
        # Ensure clean shutdown
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()