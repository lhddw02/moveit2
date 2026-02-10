import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():



    pkg_moveit_config = get_package_share_directory("panda_moveit_config")
    pkg_robot_description = get_package_share_directory("panda_description")
    urdf_name = "panda_description_gazebo.xacro"
    urdf_file = os.path.join(pkg_robot_description, f'urdf/{urdf_name}')
    srdf_file = os.path.join(pkg_moveit_config, "config", "panda.srdf")
    trajectory_execution_file = os.path.join(
        pkg_moveit_config, "config", "gripper_moveit_controllers.yaml"
    )

    # rviz config file
    default_rviz_config_path = os.path.join(pkg_robot_description, 'rviz/panda_ctrl.rviz')

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

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    panda_desc = doc.toxml()
    params = {'robot_description': panda_desc}


    # --- Build MoveIt config from panda_moveit_config ---
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit_config")
        #.robot_description(file_path= urdf_file)  
        #.robot_description_semantic(file_path= srdf_file)
        #.trajectory_execution(file_path= trajectory_execution_file)
        .robot_description(file_path="config/panda.urdf.xacro")   # MoveIt’s internal copy
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        #.moveit_cpp(file_path="config/controller_setting.yaml")
        .planning_pipelines(pipelines=["chomp" ]) #"ompl", "pilz_industrial_motion_planner"
        .to_moveit_configs()
    )

    #moveit_config.planning_pipelines["default_planning_pipeline"] = "ompl"

    # --- Nodes ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen"
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
            '-entity', 'panda',  
            '-topic', 'robot_description',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.0'], 
        output='screen')

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True},
                    {"planning_plugin": "chomp_interface/CHOMPPlanner"},
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
        get_package_share_directory("panda_gazebo"), #panda_moveit_config
        "config",
        "ros2_controllers.yaml",
    )

    initial_positions_path = os.path.join( 
        get_package_share_directory("panda_moveit_config"), "config", "initial_positions.yaml", 
    )

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

    return LaunchDescription([
        start_gazebo_cmd,
        #robot_state_publisher,
        #joint_state_publisher_node,
        spawn_entity_cmd,
        static_tf_node,
        move_group_node,
        rviz_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
    ])
