from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_control'),
        'config',
        'control.yaml'],
    )

    robot_description_command_arg = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            )
        ]
    )

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    # Nodes
    # Teleop
    teleop_node = ComposableNode(package='jackal_teleop',
                                 plugin='jackal_teleop::JackalTeleop',
                                 name='jackal_teleop_node')
    joy_node = Node(package='joy',
                    executable='joy_node',
                    output='screen')
    # control
    robot_state_node = ComposableNode(package='robot_state_publisher',
                                      plugin='robot_state_publisher::RobotStatePublisher',
                                      name='robot_state_publisher',
                                      parameters=[{'robot_description': robot_description_content}])

    jackal_node = ComposableNode(package='jackal_serial',
                                 plugin='JackalSerial::JackalSerialNode',
                                 name='jackal_serial_node',
                                 parameters=[{'dev': '/dev/jackal',
                                              'baud_rate': 115200}])

    control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description_content},
                        config_jackal_velocity_controller],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        )
    # controller_manager_node = ComposableNode(package='controller_manager',
    #                                           plugin='controller_manager::ControllerManager',
    #                                           name='controller_manager',
    #                                           parameters=[{'robot_description': robot_description_content},
    #                                                       config_jackal_velocity_controller])

    joint_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    vel_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['jackal_velocity_controller'],
        output='screen',
    )


    container = ComposableNodeContainer(
        name='PlatformComposition',
        namespace='jackal',
        package='rclcpp_components',
        executable='component_container_mt',  
        composable_node_descriptions=[teleop_node,
                                      robot_state_node,
                                      jackal_node],
        output='screen'
    )

    return LaunchDescription([robot_description_command_arg, 
                              container, 
                              joy_node,
                              control_node,
                              joint_spawner_node, 
                              vel_spawner_node])
