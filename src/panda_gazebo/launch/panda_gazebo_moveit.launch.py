import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch.event_handlers import OnProcessExit

from moveit_configs_utils import MoveItConfigsBuilder

import xacro

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'panda'
    pkg_moveit_config = get_package_share_directory("panda_moveit_config")
    
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit_config")
        .robot_description(file_path="config/panda.urdf.xacro")   # MoveIt’s internal copy
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["chomp" ]) #"ompl", "pilz_industrial_motion_planner"
        .to_moveit_configs()
    )

    # gazebo world file
    world_file = os.path.join(
        get_package_share_directory('panda_gazebo'),
        'worlds',
        'arm_on_the_table.sdf'
    )

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],        #, world_file
        output='screen',
        
    )

    # 启动了robot_state_publisher节点后，该节点会发布 robot_description 话题，话题内容是模型文件urdf的内容？
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True}, 
            moveit_config.to_dict(), 
            {"publish_frequency":15.0}],
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



    ld = LaunchDescription()
    

    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)

    ld.add_action(spawn_entity_cmd)
    ld.add_action(load_joint_state_controller)
    ld.add_action(load_arm_controller)

    return ld
