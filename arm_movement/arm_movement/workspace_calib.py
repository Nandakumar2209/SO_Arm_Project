import os
import placo
import numpy as np
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory

# Loaded from the installed share dir (not src/) so placo can resolve the
# URDF's package://arm_movement/... mesh URIs via the ROS resource index.
URDF_PATH = os.path.join(
    get_package_share_directory('arm_movement'),
    'Viz_asset', 'SO101', 'so101_new_calib.urdf'
)

robot = placo.RobotWrapper(URDF_PATH, placo.Flags.ignore_collisions)

joint_names = list(robot.joint_names())

# Get joint limits
joint_limits = []
for j in joint_names:
    lower, upper = robot.get_joint_limits(j)
    joint_limits.append((lower, upper))

samples = 5000
points = []

for _ in range(samples):
    # Sample random joint configuration
    for j, (low, high) in zip(joint_names, joint_limits):
        q = np.random.uniform(low, high)
        robot.set_joint(j, q)

    robot.update_kinematics()

    T = robot.get_T_a_b("world", "gripper_frame_link")
    pos = T[:3, 3]
    points.append(pos)

points = np.array(points)
print("max x:", np.max(points[:, 0]), "min x:", np.min(points[:, 0]))
print("max y:", np.max(points[:, 1]), "min y:", np.min(points[:, 1]))
print("max z:", np.max(points[:, 2]), "min z:", np.min(points[:, 2]))

# Plot
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(points[:,0], points[:,1], points[:,2], s=1)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.show()