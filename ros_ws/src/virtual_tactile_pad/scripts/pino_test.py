import numpy as np
from scipy.spatial.transform import Rotation
import pinocchio as pin


class RobotUtils():

    def __init__(self, urdf_path):
        self.model_pin = pin.buildModelFromUrdf(urdf_path)
        self.data_pin = self.model_pin.createData()

        self.frame_id = self.model_pin.getFrameId("panda_link8")

    ## Pinocchio basic inverse kinematic
    # R: desired rotation matrix
    # position: desired task space position
    # q0_init: initial guess, used to have a solution close to it
    # return: joint position
    def inverse_kinematics(self, R, position, q0_init = None):

        eps    = 1e-4
        IT_MAX = 1000
        DT     = 1e-1
        damp   = 1e-12

        if q0_init is None:
            q0, _ = self.sample_random_state()
        else:
            q0 = q0_init
                

        pin.forwardKinematics(self.model_pin ,self.data_pin , q0)
        pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)

        oMdes = pin.SE3.Identity()
        oMdes.rotation = R
        oMdes.translation = position
        
        i=0
        while True:
            pin.forwardKinematics(self.model_pin ,self.data_pin , q0)
            pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)
            dMi = oMdes.actInv(self.data_pin.oMf[self.frame_id])
            err = pin.log(dMi).vector
            if np.linalg.norm(err) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break
            J = pin.computeFrameJacobian(self.model_pin, self.data_pin, q0, self.frame_id)
            v = - J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
            q0 = pin.integrate(self.model_pin, q0, v*DT)
            # if not i % 100:
            #     print('%d: error = %s' % (i, err.T))
            # i += 1

        return q0

    ## Damped pseudo-inverse inverse velocity (cartesian to joint)
    # q: joint configuration
    # linear_velocity: task space linear velocity (3D)
    # angular_velocity: task space angular velocity (3D)
    # return: joint velocity
    def inverse_velocity(self, q, linear_velocity, angular_velocity):
        task_velocity = np.concatenate( (linear_velocity, angular_velocity) )

        pin.forwardKinematics(self.model_pin ,self.data_pin , q)
        pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)
        J = pin.computeFrameJacobian(self.model_pin, self.data_pin, q, self.frame_id)

        R_tot = np.zeros((6,6))
        R_tot[0:3, 0:3] = self.data_pin.oMf[self.frame_id].rotation
        R_tot[3:, 3:] = self.data_pin.oMf[self.frame_id].rotation
        J = R_tot@J
        J_damped = J@J.T + np.identity(6)*1e-5

        qdot = J.T@np.linalg.solve(J_damped, task_velocity)

        return qdot

    ## Simple conversion from joint to cartesian velocity
    # q: joint configuration
    # qdot: joint velocity
    # return: cartesian velocity (6D linear + angular)
    def forward_velocity(self, q, qdot):

        pin.forwardKinematics(self.model_pin ,self.data_pin , q)
        pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)
        J = pin.computeFrameJacobian(self.model_pin, self.data_pin, q, self.frame_id)

        R_tot = np.zeros((6,6))
        R_tot[0:3, 0:3] = self.data_pin.oMf[self.frame_id].rotation
        R_tot[3:, 3:] = self.data_pin.oMf[self.frame_id].rotation
        J = R_tot@J

        return J@qdot
    
    def normal_jacobian(self, q):

        pin.forwardKinematics(self.model_pin ,self.data_pin , q)
        pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)
        J = pin.computeFrameJacobian(self.model_pin, self.data_pin, q, self.frame_id)

        R_tot = np.zeros((6,6))
        R_tot[0:3, 0:3] = self.data_pin.oMf[self.frame_id].rotation
        R_tot[3:, 3:] = self.data_pin.oMf[self.frame_id].rotation
        J = R_tot@J

        return J

    ## Simple forward kinematic
    # q: joint configuration
    # return: tuple of rotation matrix and task position vector of the end effector
    def forward_kinematic(self, q):

        pin.forwardKinematics(self.model_pin ,self.data_pin , q)
        pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)

        return self.data_pin.oMf[self.frame_id].rotation, np.around(self.data_pin.oMf[self.frame_id].translation, 4)
    
    ## Forward classic acceleration
    def forward_classic_acc_world_aligned(self, q, qdot, qddot):

        pin.forwardKinematics(self.model_pin, self.data_pin, q, qdot, qddot)
        pin.updateFramePlacement(self.model_pin, self.data_pin, self.frame_id)
        frame_acc_aligned_classic = pin.getFrameClassicalAcceleration(self.model_pin, self.data_pin, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return frame_acc_aligned_classic
    


robot_utils = RobotUtils("/home/ros/ros_ws/src/virtual_tactile_pad/urdf/panda_arm.urdf")

def test_jacobian_calculation():
    # Create a sample joint configuration (7 joints for Panda robot)
    q = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4])
    
    # Get the Jacobian matrix
    J = robot_utils.normal_jacobian(q)
    
    # Print the Jacobian matrix
    print("Jacobian matrix (6x7):")
    print("Shape:", J.shape)
    print(np.round(J, 3))
    
    # Demonstrate how the Jacobian relates joint velocities to end-effector velocities
    # Create a sample joint velocity
    qdot = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Only moving the first joint
    
    # Calculate the resulting end-effector velocity using the Jacobian
    cartesian_vel = robot_utils.forward_velocity(q, qdot)
    
    print("\nFor joint velocities:", qdot)
    print("Resulting end-effector linear velocity (m/s):", np.round(cartesian_vel[:3], 3))
    print("Resulting end-effector angular velocity (rad/s):", np.round(cartesian_vel[3:], 3))
    
    # Verify the relationship between joint and task space velocities
    # Get current end-effector pose
    R, pos = robot_utils.forward_kinematic(q)
    print("\nCurrent end-effector position:", pos)
    print("Current end-effector orientation (rotation matrix):")
    print(np.round(R, 3))

test_jacobian_calculation()