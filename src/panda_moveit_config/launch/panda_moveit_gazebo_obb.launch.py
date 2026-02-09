import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # --- Load URDF from panda_description ---
    pkg_panda_description = get_package_share_directory("panda_description")
    urdf_file = os.path.join(pkg_panda_description, "urdf", "panda_description_gazebo.xacro")
    # doc = xacro.parse(open(urdf_file))
    # xacro.process_doc(doc)
    # robot_description = {"robot_description": doc.toxml()}

    robot_name_in_model = 'panda'
    pkg_moveit_config = get_package_share_directory("panda_moveit_config")
    
    # gazebo
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
    # gazebo

    # moveit
    # --- Build MoveIt config from panda_moveit_config ---
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit_config")
        .robot_description(file_path= urdf_file)  
        #.robot_description_semantic(file_path= srdf_file)
        #.trajectory_execution(file_path= trajectory_execution_file)
        #.robot_description(file_path="config/panda.urdf.xacro")   # MoveIt’s internal copy
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        #.moveit_cpp(file_path="config/controller_setting.yaml")
        .planning_pipelines(pipelines=["chomp" ]) #"ompl", "pilz_industrial_motion_planner"
        .to_moveit_configs()
    )

    # moveit

    # --- Nodes ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen"
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True},
                    {"planning_plugin": "chomp_interface/CHOMPPlanner"},
                    ],
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="panda_moveit_config",
        executable="arm_control_from_UI.py",
        output="both",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True},
                    ],
    )

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
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        output="screen"
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
    )

    # ros2_control node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    initial_positions_path = os.path.join( 
        get_package_share_directory("panda_moveit_config"), "config", "initial_positions.yaml", 
    )

    # Gazebo spawn Node
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

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "-c", "/controller_manager"],
    )

    ld = LaunchDescription()

    ld.add_action(start_gazebo_cmd)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    
    ld.add_action(move_group_node)
    # ld.add_action(moveit_py_node)
    ld.add_action(rviz_node)
    ld.add_action(static_tf_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(panda_arm_controller_spawner)
    ld.add_action(panda_hand_controller_spawner)


    return ld
