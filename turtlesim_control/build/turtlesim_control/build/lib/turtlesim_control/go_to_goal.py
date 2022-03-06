import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import numpy as np
import math

class TurtlesimController(Node):
    def __init__(self):
        super().__init__('turtlesim_controller')
        self.publisher = self.create_publisher(Twist,'cmd_vel',10)
        self.subscription = self.create_subscription(Pose,'pose',self.pose_callback,10)
        self.pose = Pose()
        self.subscription
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
    def pose_callback(self,msg):
        #self.get_logger().info('I heard: "%s"' % msg.x)
        self.pose = msg
    def timer_callback(self):
        goal = [2,3]
        v,w = self.control(goal)
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher.publish(msg)
        #self.get_logger().info('Publishing Linear Velocity: "%s"' % msg.linear.x)
        #self.get_logger().info('Publishing Angular Velocity: "%s"' % msg.angular.z)
    def control(self,goal):
        dp = np.array(goal)-np.array([self.pose.x,self.pose.y])
        if np.linalg.norm(dp)<0.1:
            v = 0.0
            w = 0.0
        else:
            v = 1.0
            e = math.atan2(dp[1],dp[0])-self.pose.theta
            K = 1.0
            w = K*math.atan2(math.sin(e),math.cos(e)) 
        return v,w
def main(args=None):
    rclpy.init(args=args)
    turtlesimController = TurtlesimController()
    rclpy.spin(turtlesimController)
    turtlesimController.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
