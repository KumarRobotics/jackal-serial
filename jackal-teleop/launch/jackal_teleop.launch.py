from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='jackal_teleop_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  
        composable_node_descriptions=[
            ComposableNode(
                package='jackal_teleop',
                plugin='jackal_teleop::JetiJoy',
                name='jeti_joy'
            ),
            ComposableNode(
                package='jackal_teleop',
                plugin='jackal_teleop::JackalTeleop',
                name='jackal_teleop_node'
            ),
        ],
        output='screen'
    )

    return LaunchDescription([container])
