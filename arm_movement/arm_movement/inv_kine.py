import os
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import placo
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

# The gripper (finger) joint is attached to gripper_link via a fixed joint,
# not gripper_frame_link, so it has no effect on end-effector pose and must
# not be part of the IK unknowns.
GRIPPER_JOINT = 'gripper'


class InvKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        # Declare parameters
        self.declare_parameter('target_position',[0.2,0.3,0.3])
        self.declare_parameter('target_orientation',[0.0,0.0,np.pi])
        # Get parameters
        target_position = self.get_parameter('target_position').value
        target_orientation = self.get_parameter('target_orientation').value
        self.target_position = np.array(target_position, dtype=np.float64)
        self.target_orientation = np.array(target_orientation, dtype=np.float64)
        self.publisher = self.create_publisher(
            JointState,
            '/ik_joint_states',
            10
        )
        # Robot model setup
        urdf_path = os.path.join(
            get_package_share_directory('arm_movement'),
            'Viz_asset', 'SO101', 'so101_new_calib.urdf'
        )
        self.robot = placo.RobotWrapper(urdf_path, placo.Flags.ignore_collisions)
        self.joint_names = list(self.robot.joint_names())
        # Joints that are actually solved for in IK (everything but the gripper finger)
        self.ik_joint_names = [j for j in self.joint_names if j != GRIPPER_JOINT]
        self.ik_lower_bounds = np.array(
            [self.robot.get_joint_limits(j)[0] for j in self.ik_joint_names])
        self.ik_upper_bounds = np.array(
            [self.robot.get_joint_limits(j)[1] for j in self.ik_joint_names])
        self.__set_default_robot_state()
        # Compute and publish joint angles based on the target position and orientation
        self.inv_kine_callback()

    def __set_default_robot_state(self):
        # Set robot to a default configuration (e.g., all joints to zero)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = [0.0] * len(self.joint_names)
        self.set_joint_angles(self.joint_names, joint_state_msg.position)
        self.publisher.publish(joint_state_msg)
        time.sleep(0.5)  # Allow time for the robot to update

    def inv_kine_callback(self):
        # Extract target position and orientation from the PoseStamped message
        joint_angles = self.inverse_kinematics(self.target_position, self.target_orientation)
        self.get_logger().info(f'Publishing joint angles: {self.joint_names} with values {joint_angles}')
        # Publish the computed joint angles
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_angles
        self.set_joint_angles(self.joint_names, joint_angles)
        self.publisher.publish(joint_state_msg)
        time.sleep(0.5)  # Allow time for the robot to update



    def inverse_kinematics(self, target_position, target_orientation):
        # Build target transform
        target = self.make_target(target_position, target_orientation)
        p_des = target[:3, 3]
        R_des = target[:3, :3]

        # Initial guess: zero is within bounds for every joint on this arm
        q0 = np.clip(np.zeros(len(self.ik_joint_names)), self.ik_lower_bounds, self.ik_upper_bounds)

        def residual(q):
            # FK at q (gripper finger joint is left at its current value; it
            # doesn't affect gripper_frame_link's pose)
            self.set_joint_angles(self.ik_joint_names, q)
            T_fk = self.robot.get_T_a_b("world", "gripper_frame_link")
            p_fk = T_fk[:3, 3]
            R_fk = T_fk[:3, :3]

            # Position residual (meters)
            r_pos = (p_fk - p_des)

            # Orientation residual (rotation vector in radians)
            R_err = R_des.T @ R_fk
            r_ori = R.from_matrix(R_err).as_rotvec()

            # If you want position-only IK, return r_pos only
            return np.hstack([r_pos, r_ori])

        # trf supports bounds so the solution can't leave the URDF joint limits
        res = least_squares(
            residual, q0, method="trf",
            bounds=(self.ik_lower_bounds, self.ik_upper_bounds), max_nfev=200)

        self.get_logger().info(f"IK success={res.success}, cost={res.cost}, status={res.status}")
        if not res.success:
            self.get_logger().warn('IK solver did not converge; target may be unreachable')

        solved = dict(zip(self.ik_joint_names, res.x.tolist()))
        solved[GRIPPER_JOINT] = self.robot.get_joint(GRIPPER_JOINT)
        return [solved[name] for name in self.joint_names]

    def set_joint_angles(self, joint_names, joint_angles):
        for name, q in zip(joint_names, joint_angles):
            try:
                self.robot.set_joint(name, float(q))  # common pattern
            except Exception:
                self.get_logger().error(f'Failed to set joint {name} to angle {q}')
        self.robot.update_kinematics()

    def make_target(self, position, orientation):
        target = np.eye(4)
        target[:3, 3] = position
        target[:3, :3] = R.from_euler('xyz', orientation).as_matrix()
        return target
def main(args=None):
    rclpy.init(args=args)
    Inverse_kinematics_node = InvKinematicsNode()
    rclpy.spin(Inverse_kinematics_node)
    Inverse_kinematics_node.destroy_node()
    rclpy.shutdown()

