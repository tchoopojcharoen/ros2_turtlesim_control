#!/usr/bin/env python3
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class BaseController(Node):
    def __init__(self,name):
        super().__init__(name)
        self.isEnable = False
        self.command_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.pose_subscription = self.create_subscription(Pose,'/pose',self.pose_callback,10)
        self.pose = Pose()
        self.goal = np.array([2.0,3.0])
        self.declare_parameters(namespace='',parameters=[('gain',5.0),])
        
    def timer_callback(self):
        if self.isEnable:
            msg = self.control()
            self.command_publisher.publish(msg)
        else:
            current_position = np.array([self.pose.x,self.pose.y])
            dp = self.goal-current_position
            if np.linalg.norm(dp)>0.1:
                self.departure_callback()
    def pose_callback(self,msg):
        self.pose = msg
    def control(self):
        msg = Twist()
        current_position = np.array([self.pose.x,self.pose.y])
        dp = self.goal-current_position
        e = np.arctan2(dp[1],dp[0])-self.pose.theta
        K = self.get_parameter('gain').get_parameter_value().double_value
        w = K*np.arctan2(np.sin(e),np.cos(e))
        if np.linalg.norm(dp)>0.1:
            v = 1.0
        else:
            v = 0.0
            w = 0.0
            self.arrival_callback()

        msg.linear.x = v
        msg.angular.z = w
        return msg
    def arrival_callback(self):
        pass
    def departure_callback(self):
        pass