import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch.event_handlers import OnProcessExit

import xacro

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'panda'
    package_name = 'panda_description'
    urdf_name = "panda_description_gazebo.xacro"
    

    #pkg_share = FindPackageShare(package=package_name).find(package_name) 
    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    
    
    # rviz config file
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/panda_ctrl.rviz')

    # rviz
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    # gazebo world file
    world_file = os.path.join(
        get_package_share_directory('panda_gazebo'),
        'worlds',
        'test_world.world'
    )

    


    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
        
    )


    # 因为 urdf文件中有一句 $(find mybot) 需要用xacro进行编译一下才行
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    panda_desc = doc.toxml()
    params = {'robot_description': panda_desc}

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容？
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,  
            '-topic', 'robot_description',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.0'], 
        output='screen')

    # 关节状态发布器
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'panda_arm_controller'],
        output='screen'
    )

    ###moveit
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()
    move_group_node = generate_move_group_launch(moveit_config)
    ###moveit

    rviz_config = os.path.join(
        get_package_share_directory("panda_moveit_config"),
        "launch",
        "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            {'use_sim_time': True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        output="screen"
    )

    ld = LaunchDescription()

    # ld.add_action(rviz_arg)
    # ld.add_action(rviz_node)

    ld.add_action(load_joint_state_controller)
    ld.add_action(load_arm_controller)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    # ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)

    return ld
