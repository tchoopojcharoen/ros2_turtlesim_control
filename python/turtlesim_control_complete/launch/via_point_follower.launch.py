from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node

import os
def strSplitNodeName(strNodeList):
    list = strNodeList.split("\n")
    list = list[0:-1]
    return list

def generate_launch_description():
    strNodeList = os.popen('ros2 node list').read()
    listNodeAvailable = strSplitNodeName(strNodeList)
    turtlesimRunning = False
    for node in listNodeAvailable:
        if node=="/turtlesim":
            turtlesimRunning = True
    
    robot_turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[
            {"background_b": 100},
            {"background_g": 20},
            {"background_r": 0}
        ],
    )
    
    spawn_turtle = ExecuteProcess(cmd=[['ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"']],shell=True)

    controller = Node(
        package='turtlesim_control',
        executable='turtlesim_controller',
        name='controller',
        parameters=[
            {"gain": 1.0}
        ],
        remappings=[
            ('/pose','/turtle1/pose'),
            ('/cmd_vel','/turtle1/cmd_vel')
        ]
    )

    scheduler = Node(
        package='turtlesim_control',
        executable='turtlesim_scheduler',
        name='scheduler',
    )

    if turtlesimRunning:
        nodes_to_run  = [controller,scheduler]
    else:
        nodes_to_run  = [robot_turtle,spawn_turtle,controller,scheduler]

    return LaunchDescription(nodes_to_run)
