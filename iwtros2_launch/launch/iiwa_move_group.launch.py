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
    planning_pipelines_config = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_pkg),
            "config",
            "planning_pipelines_config.yaml",
        ]
    )
    ompl_planning_config = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_pkg),
            "config",
            "ompl_planning.yaml",
        ]
    )

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
            planning_pipelines_config,
            ompl_planning_config,
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
