from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    ThisLaunchFileDir,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Turn Off and on the Gazebo GUI",
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
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
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
            "robot_ip",
            default_value="192.170.10.2",
            description="Robot IP of FRI interface",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="30200",
            description="Robot port of FRI interface.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    moveit_config_pkg = LaunchConfiguration("moveit_config_pkg")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_rviz = LaunchConfiguration("start_rviz")
    use_sim = LaunchConfiguration("use_sim")
    gui = LaunchConfiguration("gui")
    use_planning = LaunchConfiguration("use_planning")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")

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
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_port:=",
            robot_port,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "iiwa_controllers.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        condition=UnlessCondition(use_sim),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false", "gui": gui}.items(),
        condition=IfCondition(use_sim),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "kuka_iiwa7"],
        output="screen",
        condition=IfCondition(use_sim),
    )

    iiwa_move_group_planning_launch_file = PathJoinSubstitution(
        [FindPackageShare("iwtros2_launch"), "launch", "iiwa_move_group.launch.py"]
    )
    iiwa_move_group_planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([iiwa_move_group_planning_launch_file]),
        launch_arguments={
            "description_package": description_package,
            "moveit_config_pkg": moveit_config_pkg,
            "description_file": description_file,
            "prefix": prefix,
            "start_rviz": start_rviz,
        }.items(),
        condition=IfCondition(use_planning),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            robot_description,
        ],
        condition=UnlessCondition(use_planning),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
    )

    external_torque_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ets_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_sim),
    )

    # Delay joint state broadcaster after spawning the entity
    # delay_joint_state_broadcaster_after_spawn_entity = delayed_spwan_controller = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=node_controller_manager,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     ),
    #     condition=IfCondition(use_sim)
    # )

    delay_joint_state_broadcaster_after_spawn_entity = TimerAction(
        period=60.0, actions=[joint_state_broadcaster_spawner]
    )

    # Delay `joint_state_broadcaster` after control_node
    delay_joint_state_broadcaster_after_controller_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_controller_manager,
            on_exit=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawn_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        ),
    )

    nodes = [
        gazebo,
        node_robot_state_publisher,
        node_controller_manager,
        iiwa_move_group_planning,
        spawn_entity,
        delay_joint_state_broadcaster_after_spawn_entity,
        delay_joint_state_broadcaster_after_controller_manager,
        delay_rviz_after_joint_state_broadcaster_spawner,
        external_torque_broadcaster_spawner,
        delay_robot_controller_spawn_after_joint_state_broadcaster,
    ]

    return LaunchDescription(declared_arguments + nodes)
