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
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.inv_kine_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        # Robot buffers and state
        self.last_joint_angles = None
        self.last_target_angles = None
        # Robot model and kinematics solver setup
        self.robot = placo.RobotWrapper("src/arm_movement/Viz_asset/SO101/so101_new_calib.urdf",placo.Flags.ignore_collisions)
        self.joint_names = list(self.robot.joint_names())
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.ee_task = self.solver.add_frame_task("gripper_frame_link", np.eye(4))
        self.ee_task.configure("effector", "soft", 1.0,1.0)
    def inv_kine_callback(self, msg):
        # Extract target position and orientation from the PoseStamped message
        target_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        target_orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        joint_angles = self.inverse_kinematics(target_position, target_orientation)
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
        joint_angles = []
        for joint in self.joint_names:
            joint_angles.append(self.robot.get_joint(joint))
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

