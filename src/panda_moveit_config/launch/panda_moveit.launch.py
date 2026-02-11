import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # --- Load URDF from panda_description ---
    # pkg_panda_description = get_package_share_directory("panda_description")
    # urdf_file = os.path.join(pkg_panda_description, "urdf", "panda_description_gazebo.xacro")
    # doc = xacro.parse(open(urdf_file))
    # xacro.process_doc(doc)
    # robot_description = {"robot_description": doc.toxml()}

    pkg_moveit_config = get_package_share_directory("panda_moveit_config")
    urdf_file = os.path.join(pkg_moveit_config, "config", "panda.urdf.xacro")
    srdf_file = os.path.join(pkg_moveit_config, "config", "panda.srdf")
    trajectory_execution_file = os.path.join(
        pkg_moveit_config, "config", "gripper_moveit_controllers.yaml"
    )


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
            {'use_sim_time': True},
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
        parameters=[ moveit_config.robot_description, ros2_controllers_path, {'use_sim_time': True}],
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
        #robot_state_publisher,
        #static_tf_node,
        move_group_node,
        rviz_node,
        ros2_control_node,
        #joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,
    ])
