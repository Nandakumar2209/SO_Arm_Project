import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_movement')
    urdf_path = os.path.join(pkg_share, 'Viz_asset', 'SO101', 'so101_new_calib.urdf')
    rviz_config = os.path.join(pkg_share, 'resource','arm.rviz')

    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config],
    output='screen'
    )

    return LaunchDescription([jsp_gui_node, rsp_node, rviz_node])