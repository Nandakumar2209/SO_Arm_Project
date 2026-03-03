import placo
import numpy as np
from scipy.spatial.transform import Rotation as R

robot = placo.RobotWrapper(
    "src/arm_movement/Viz_asset/SO101/so101_new_calib.urdf",
    placo.Flags.ignore_collisions
)

ee_frame = "gripper_frame_link"

joint_names = list(robot.joint_names())

# Try to get limits; if your placo version doesn't support robot.joint_limits(),
# see the note below.
limits = []
for j in joint_names:
    low, high = robot.get_joint_limits(j)   # if this exists in your version
    limits.append((low, high))

N = 20000
poses = []  # each row: [x,y,z,qx,qy,qz,qw]

for _ in range(N):
    for j, (low, high) in zip(joint_names, limits):
        robot.set_joint(j, float(np.random.uniform(low, high)))

    robot.update_kinematics()

    T = robot.get_T_a_b("world", ee_frame)
    pos = T[:3, 3]
    quat = R.from_matrix(T[:3, :3]).as_quat()  # [x,y,z,w]

    poses.append([pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]])

poses = np.array(poses)
print("Saved poses:", poses.shape)

# Example: print rough bounds of reachable position
mins = poses[:, :3].min(axis=0)
maxs = poses[:, :3].max(axis=0)
print("Position bounds x,y,z min:", mins, "max:", maxs)