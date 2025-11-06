import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='base',
        description='Configuration type for Jackal'
    )
    
    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='/',
        description='Namespace for the robot'
    )
    
    # Get package directories
    jackal_description_dir = get_package_share_directory('jackal_description')
    
    # Build the xacro command
    xacro_file = os.path.join(jackal_description_dir, 'urdf', 'jackal.urdf.xacro')
    config_path = os.path.join(jackal_description_dir, 'urdf', 'configs', LaunchConfiguration('config'))
    
    robot_description_content = Command([
        'python3 ',
        os.path.join(jackal_description_dir, 'scripts', 'env_run'),
        ' ',
        config_path,
        ' ',
        'xacro',
        ' ',
        xacro_file,
        ' ',
        'ns:=', LaunchConfiguration('ns')
    ])
    
    # Create a node to publish robot_description parameter
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('ns'),
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )
    
    return LaunchDescription([
        config_arg,
        ns_arg,
        robot_state_publisher_node
    ])
