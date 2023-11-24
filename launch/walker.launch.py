from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.actions import LogInfo
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown

def generate_launch_description():
    record_bag = DeclareLaunchArgument(
        'record_bag',
        default_value=TextSubstitution(text="1"),
        description='Enable/Disable rosbag recording'
    )
    roomba_node = Node(
        package='mini_roomba',
        executable='walker_node',
        name='walker_node',
        output='screen',
        parameters=[{"record_bag": LaunchConfiguration('record_bag')}],
        remappings=[],
    )
    
    ros2_bag_start = Node(
        package='rosbag2',
        executable='record',
        name='rosbag_record',
        output='screen',
        arguments=[
            '--all',
            '--exclude', '/camera/*',  # Exclude camera topics
            '--output', 'your_bag_file.bag'
        ],
        condition=IfCondition(LaunchConfiguration('record_bag'))
    )
    
    return LaunchDescription([record_bag, roomba_node])
    
