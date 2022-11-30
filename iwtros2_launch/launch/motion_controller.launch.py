from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder

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
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="iiwa7.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="iiwa7",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_planning",
            default_value="true",
            description="Enable MoveIt Planning Interface",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="iiwa_arm_controller",
            description="Which controller to use.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.73',
            description='Robot IP of FRI interface',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_port',
            default_value='30200',
            description='Robot port of FRI interface.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_ip",
            default_value="192.168.1.160",
            description="Gripper IP address",
        ),
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "attach_gripper",
            default_value="true",
            description="Condition to run gripper hardware driver",
        ),
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    moveit_config_pkg = LaunchConfiguration("moveit_config_pkg")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration("use_sim")
    use_planning = LaunchConfiguration("use_planning")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    gripper_ip = LaunchConfiguration('gripper_ip')
    attach_gripper = LaunchConfiguration('attach_gripper')

    moveit_config_builder = (
        MoveItConfigsBuilder(
            "iiwa7", 
            package_name='iiwa7_moveit_config')
            .robot_description_semantic(file_path='config/iiwa.srdf')
            .trajectory_execution(file_path='config/moveit_controllers.yaml')
            .robot_description_kinematics(file_path='config/kinematics.yaml')
            .to_moveit_configs()
        )
    robot_description_planning_joint_limits = PathJoinSubstitution(
        [FindPackageShare(moveit_config_pkg), "config", "joint_limits.yaml",]
        )

    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }
    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare(moveit_config_pkg), "config", "planning_pipelines_config.yaml",
        ]
    )
    ompl_planning_config = PathJoinSubstitution([
            FindPackageShare(moveit_config_pkg), "config", "ompl_planning.yaml",
        ]
    )

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
            ' ',
            'robot_ip:=',
            robot_ip,
            ' ',
            'robot_port:=',
            robot_port,
            ' ',
            'use_fake_hardware:=',
            use_fake_hardware,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    node = Node(
        package='iwtros2_launch',
        executable='iiwa7_manipulation_test_node', #'iiwa7_manipulation_node',
        name='iiwa_motion_controller',
        namespace='move_group',
        parameters=[
            robot_description,
            moveit_config_builder.to_dict(), 
            robot_description_planning_joint_limits,
            move_group_capabilities,
            planning_pipelines_config,
            ompl_planning_config,
        ],
        # remappings=[
        #     ('/move_group/joint_states', '/joint_states'),
        # ],
    )

    gripper_driver_launch_file = PathJoinSubstitution([FindPackageShare('wsg50_driver'), "launch" , "gripper.launch.py"])
    gripper_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gripper_driver_launch_file]),
        launch_arguments={
            'gripper_ip': gripper_ip,
        }.items(),
        condition=IfCondition(attach_gripper),
    )

    delay_iiwa_motion_controller = TimerAction(
        period=10.0,
        actions=[node],
    )

    joint_remap_node = Node(
        package='iwtros2_launch',
        executable='joint_state_combine_node', #'iiwa7_manipulation_node',
        name='joint_state_remap_node',
    )

    nodes = [
        gripper_driver_node,
        delay_iiwa_motion_controller, 
        joint_remap_node,
    ]

    return LaunchDescription(
        declared_arguments + nodes 
    )