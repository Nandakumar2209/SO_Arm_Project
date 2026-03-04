import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import placo 
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares





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
            '/joint_states',
            10
        )
        # Robot buffers and state
        self.last_joint_angles = None
        self.last_target_angles = None
        # Robot model and kinematics solver setup
        self.robot = placo.RobotWrapper("src/arm_movement/Viz_asset/SO101/so101_new_calib.urdf",placo.Flags.ignore_collisions)
        self.joint_names = list(self.robot.joint_names())
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        T = self.__set_default_robot_state()
        # self.ee_task = self.solver.add_frame_task("gripper_frame_link",T)
        # self.ee_task.configure("gripper_frame_link", "hard", 2.0,0.0)
        # Compute and publish joint angles based on the target position and orientation
        self.inv_kine_callback()

    def __set_default_robot_state(self):
        # Set robot to a default configuration (e.g., all joints to zero)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = [0.0] * len(self.joint_names)
        for joint in self.joint_names:
            self.robot.set_joint(joint, 0.0)
        self.robot.update_kinematics()
        self.publisher.publish(joint_state_msg)
        time.sleep(0.5)  # Allow time for the robot to update
        T = self.robot.get_T_a_b("world", "gripper_frame_link")
        return T
    def inv_kine_callback(self):
        # Extract target position and orientation from the PoseStamped message
        joint_angles = self.inverse_kinematics(self.target_position, self.target_orientation)
        self.get_logger().info(f'Publishing joint angles: {self.joint_names} with values {joint_angles}')
        # Publish the computed joint angles
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_angles
        self.set_joint_angles(joint_angles)
        self.publisher.publish(joint_state_msg)
        time.sleep(0.5)  # Allow time for the robot to update



    def inverse_kinematics(self, target_position, target_orientation):
        # Build target transform
        target = self.make_target(target_position, target_orientation)
        p_des = target[:3, 3]
        R_des = target[:3, :3]

        # Initial guess (zeros is okay, but better: current joints)
        q0 = np.zeros(len(self.joint_names), dtype=np.float64)

        def residual(q):
            # FK at q
            self.set_joint_angles(q)
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

        # Solve (LM works well for small residual problems)
        res = least_squares(residual, q0, method="lm", max_nfev=200)

        self.get_logger().info(f"IK success={res.success}, cost={res.cost}, status={res.status}")
        return res.x.tolist()



    def set_joint_angles(self, joint_angles):
        for name, q in zip(self.joint_names, joint_angles):
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

