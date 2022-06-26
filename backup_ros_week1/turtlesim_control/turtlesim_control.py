from queue import Empty
import rclpy
import math
import numpy as np 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

class Controller(Node):
    
    def __init__(self):
        super().__init__('Controller')
        self.cmd_pub =  self.create_publisher(Twist,'/turtle1/cmd_vel', 10)  # QOS
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose',self.subscriptions_callback, 10)
        self.pose_sub2 = self.create_subscription(Pose, '/turtle2/pose',self.subscriptions2_callback, 10)
        self.pose = Pose()
        self.pose2 = Pose()
        self.rand_goal_service = self.create_service(Empty,'/random_goal', self.random_goal_callback )
        self.goal = np.array([2.0,3.0])
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def subscriptions2_callback(self,msg):
       
        self.pose2 = msg
        self.goal = [self.pose2.x,self.pose2.y]
    
    def subscriptions_callback(self,msg):
        self.pose = msg
    
    def timer_callback(self):
        msg = self.control()
        self.cmd_pub.publish(msg)

    def control(self):
        msg = Twist()
        current_position = np.array([self.pose.x, self.pose.y])
        dp = self.goal - current_position
        e = np.arctan2(dp[1],dp[0]) - self.pose.theta
        Kp = 2.5
        w = Kp * np.arctan2(np.sin(e), np.cos(e))
        if np.linalg.norm(dp) > 0.1 :
            v= 1.25
        else :
            v= 0
        msg.angular.z = float(w)
        msg.linear.x = float(v) 
        return msg
    
    def random_goal_callback(self, req, res):
        self.goal = 9 * np.random.rand(2)+ 0.5

        return res
        
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__ == "__nain__" :
    main()
  
        
        
