from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    scheduler = Node(
        package='turtlesim_control',
        executable='scheduler.py',
        name = 'scheduler',
    )
    via_point_follower = Node(
        package='turtlesim_control',
        executable='via_point_follower.py',
        name = 'via_point_follower',
        remappings=[
            ('/pose','turtle1/pose'),
            ('/cmd_vel','turtle1/cmd_vel'),
        ]
    )
    turtle_following_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlesim_control'),
                'launch',
                'turtle_following.launch.py'
            ])
        ]),
        launch_arguments={
            'new_background_r': '100',
        }.items()
    )

    return LaunchDescription([scheduler,via_point_follower,turtle_following_launch])
