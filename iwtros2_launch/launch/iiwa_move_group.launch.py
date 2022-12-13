from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="iiwa7_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_pkg",
            default_value="iiwa7_moveit_config",
            description="Description Moveit package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="iiwa.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start Simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    moveit_config_pkg = LaunchConfiguration("moveit_config_pkg")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_rviz = LaunchConfiguration("start_rviz")
    # namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration("use_sim")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_sim:=",
            use_sim,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    moveit_config_builder = (
        MoveItConfigsBuilder("iiwa7", package_name="iiwa7_moveit_config")
        .robot_description_semantic(file_path="config/iiwa.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    robot_description_planning_joint_limits = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_pkg),
            "config",
            "joint_limits.yaml",
        ]
    )

    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }
    move_group_request_adapters = {
        "request_adapters": """default_planner_request_adapters/AddRuckigTrajectorySmoothing \
           default_planner_request_adapters/AddTimeOptimalParameterization"""
    }
    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare(moveit_config_pkg), "config", "planning_pipelines_config.yaml",
        ]
    )
    ompl_planning_config = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_pkg),
            "config",
            "ompl_planning.yaml",
        ]
    )
    
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            moveit_config_builder.to_dict(),
            robot_description_planning_joint_limits,
            move_group_capabilities,
            move_group_request_adapters,
            planning_pipelines_config,
            ompl_planning_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("iwtros2_launch"), "config", "iwtros2.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            moveit_config_builder.to_dict(),
            robot_description_planning_joint_limits,
            planning_pipelines_config,
            ompl_planning_config,
        ],
        condition=IfCondition(start_rviz),
    )

    nodes = [
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(nodes + declared_arguments)
