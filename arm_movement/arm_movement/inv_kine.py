import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import placo 
import numpy as np
from scipy.spatial.transform import Rotation as R



class InvKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
                # Declare parameters
        self.declare_parameter('target_position',[0.2,-3.0,0.2])
        self.declare_parameter('target_orientation',[0,0,0,1])
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
        self.ee_task = self.solver.add_frame_task("gripper_frame_link",T)
        self.ee_task.configure("effector", "soft", 2.0,0.0)
        # Compute and publish joint angles based on the target position and orientation
        self.inv_kine_callback()

    def __set_default_robot_state(self):
        # Set robot to a default configuration (e.g., all joints to zero)
        for joint in self.joint_names:
            self.robot.set_joint(joint, 0.0)
        self.robot.update_kinematics()
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
        try:
            for name, q in zip(self.joint_names, joint_angles):
                try:
                    self.robot.set_joint(name, float(q))  # common pattern
                except Exception:
                    self.get_logger().error(f'Failed to set joint {name} to angle {q}')
            self.publisher.publish(joint_state_msg)
            self.robot.update_kinematics()
        except Exception as e:
            self.get_logger().error(f'Failed to publish joint state: {e}')

    def inverse_kinematics(self, target_position, target_orientation):
        self.get_logger().info(f'Computing inverse kinematics for target position: {target_position}, orientation: {target_orientation}')
        target = self.make_target(target_position, target_orientation)
        self.ee_task.T_world_frame = target
        self.solver.solve(True)
        self.robot.update_kinematics()   
        T_fk = self.robot.get_T_a_b('world', 'gripper_frame_link')
        self.get_logger().info(f"FK T_pos = {T_fk[:3, 3]}, Target T = {target[:3, 3]}")
        self.get_logger().info(f"T error = {np.linalg.norm(T_fk.T@target)}") 
        joint_angles = []
        for joint in self.joint_names:
            joint_angles.append(np.round(self.robot.get_joint(joint), decimals=2))
        return joint_angles
    def make_target(self, position, orientation):
        target = np.eye(4)
        target[:3, 3] = position
        target[:3, :3] = R.from_quat(orientation).as_matrix()
        return target
def main(args=None):
    rclpy.init(args=args)
    Inverse_kinematics_node = InvKinematicsNode()
    rclpy.spin(Inverse_kinematics_node)
    Inverse_kinematics_node.destroy_node()
    rclpy.shutdown()

