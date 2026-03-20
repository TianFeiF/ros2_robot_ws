from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get URDF via xacro from the robot_new_moveit package
    # Use the full URDF Xacro file which includes both robot description and ros2_control config
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_new_moveit"), "config", "robot_new.urdf.xacro"]
            ),
        ],
        on_stderr='warn'
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Use the controller config from this package
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_new_moveit"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    # Node that handles the control loop
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # Robot State Publisher (loads the URDF)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawner for joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner for plan group controller
    plan_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["plan_group_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawner for hand group controller
    hand_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_group_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawner for base velocity controller
    base_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawner for base effort controller
    base_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_effort_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[plan_group_controller_spawner, hand_group_controller_spawner, base_velocity_controller_spawner, base_effort_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
