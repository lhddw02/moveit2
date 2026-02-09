import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to your robot description and MoveIt config
    pkg_share = get_package_share_directory('panda_moveit_config')
    urdf_file = os.path.join(pkg_share, 'urdf', 'panda.urdf.xacro')
    srdf_file = os.path.join(pkg_share, 'config', 'panda.srdf')
    kinematics_yaml = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    ompl_planning_yaml = os.path.join(pkg_share, 'config', 'ompl_planning.yaml')
    controllers_yaml = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')

    # Launch arguments
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'moveit.rviz')
    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file,
        description='RViz config file'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}],
        output='screen'
    )

    # MoveGroup node (the core of MoveIt)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': open(urdf_file).read()},
            {'robot_description_semantic': open(srdf_file).read()},
            kinematics_yaml,
            ompl_planning_yaml,
            controllers_yaml,
        ]
    )

    # RViz with MoveIt plugin
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        move_group_node,
        rviz_node
    ])
