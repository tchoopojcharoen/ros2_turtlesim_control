#!/usr/bin/env python3
from python_lib.base_controller import BaseController
import numpy as np
import rclpy

from std_srvs.srv import Empty
from turtlesim_interfaces.srv import SetGoal

class ViaPointFollower(BaseController):
    def __init__(self):
        super().__init__('via_point_follower')
        self.set_goal_service = self.create_service(SetGoal,'/set_goal',self.set_goal_callback)
        self.enable_service = self.create_service(Empty,'/enable',self.enable_callback)
        self.enable_client = self.create_client(Empty, '/notify_arrival')
        
    def set_goal_callback(self,request,response):
        self.goal = np.array([request.position.x,request.position.y])
        
        self.get_logger().info(f'Goal has been reassigned.')
        return response
    def enable_callback(self,request,response):
        self.isEnable = True
        return response
    def send_notify_request(self):
        req = Empty.Request()
        self.future = self.enable_client.call_async(req)
    def arrival_callback(self):
        self.send_notify_request()
        self.isEnable = False
        self.get_logger().info("Arrived at the goal.")
    def departure_callback(self):
        self.isEnable = True
        self.get_logger().info("Goal has been updated.")
def main(args=None):
    rclpy.init(args=args)
    via_point_follower = ViaPointFollower()
    rclpy.spin(via_point_follower)
    via_point_follower.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
