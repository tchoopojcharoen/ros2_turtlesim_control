from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
import os


def generate_launch_description():
    new_background_r = LaunchConfiguration('new_background_r')
    new_back_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value = '255'
    )
    robot_turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[
            {"background_b": 100},
            {"background_g": 20},
            {"background_r": new_background_r}
        ],
    )
    
    config = os.path.join(get_package_share_directory('turtlesim_control'),'config','follower_config.yaml')
    spawn_turtle = ExecuteProcess(cmd=[['ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"']],shell=True)

    turtle_follower = Node(
        package='turtlesim_control',
        executable='turtle_follower',
        name='turtle_follower',
        parameters=[
            config
        ],
        remappings=[
            ('/pose','/turtle2/pose'),
            ('/cmd_vel','/turtle2/cmd_vel'),
            ('/goal','/turtle1/pose')
        ]
    )
    nodes_to_run  = [new_back_r_launch_arg,robot_turtle,spawn_turtle,turtle_follower]
    return LaunchDescription(nodes_to_run)
