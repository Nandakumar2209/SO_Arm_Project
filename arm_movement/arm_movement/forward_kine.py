import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import placo 
import numpy as np
from scipy.spatial.transform import Rotation as R


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.robot = placo.RobotWrapper("arm_movement/Viz_asset/SO101/so101_new_calib.urdf")

    def joint_state_callback(self, msg):
        # Extract joint angles from the message
        joint_names = msg.name
        joint_angles = msg.position
        if len(joint_angles) < 6:
            self.get_logger().error('Received joint state with insufficient angles')
            return
        
        # Compute forward kinematics
        ee_position, ee_orientation = self.forward_kinematics(joint_angles)
        
        # Log the end-effector position
        self.get_logger().info(f'End-effector position: x={ee_position[0]:.2f}, y={ee_position[1]:.2f}, z={ee_position[2]:.2f}')
        self.get_logger().info(f'End-effector orientation: roll={ee_orientation[0]:.2f}, pitch={ee_orientation[1]:.2f}, yaw={ee_orientation[2]:.2f}')

    def forward_kinematics(self, joint_angles):
        self.get_logger().info(f'Computing forward kinematics for joint angles: {joint_angles}')
        self.get_logger().info(f'Loading robot model: {self.robot}')
        robot_Tf_matrix = self.robot.get_T_a_b('base_link', 'gripper_frame_link')
        self.get_logger().info(f'Robot transformation matrix: {robot_Tf_matrix}')
        ee_position = robot_Tf_matrix[:3, 3]
        ee_orientation = R.from_matrix(robot_Tf_matrix[:3, :3]).as_euler('xyz')
        self.robot.update_kinematics()
        return (ee_position, ee_orientation)

def main(args=None):
    rclpy.init(args=args)
    forward_kinematics_node = ForwardKinematicsNode()
    rclpy.spin(forward_kinematics_node)
    forward_kinematics_node.destroy_node()
    rclpy.shutdown()

