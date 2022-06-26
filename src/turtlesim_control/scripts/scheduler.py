#!/usr/bin/python3
from urllib import response
import numpy as np
from requests import request
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim_interfaces.srv import RandGoal,SetGoal

class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler')
        self.rand_goal_service = self.create_service(RandGoal,'/rand_goal',self.rand_goal_callback)
        self.enable_client = self.create_client(Empty, '/enable')
        self.set_goal_client = self.create_client(SetGoal, '/set_goal')
        self.via_point = np.array([[2.0,5.0,8.0,1.0,9.0,2.0,],[1.0,9.0,1.0,6.0,6.0,1.0]])
        self.num_via_points = self.via_point.shape[1]
        self.idx = 0
        self.current_goal = Point()
        self.current_goal.x = self.via_point[0][self.idx]
        self.current_goal.y = self.via_point[1][self.idx]
        self.notify_arrival_service = self.create_service(Empty,'/notify_arrival',self.notify_arrival_callback)
        while not self.set_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set goal server is not avaliable')
        self.send_set_goal_request(self.current_goal)
        
        while not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set send_enable server is not avaliable')  
        
        self.send_enable_request()
        
    def notify_arrival_callback(self,request,response):
        self.idx += 1 
        print(self.idx )
        if self.idx < self.num_via_points:
            self.current_goal.x = self.via_point[0][self.idx]
            self.current_goal.y = self.via_point[1][self.idx]
            self.send_set_goal_request(self.current_goal)
            self.send_enable_request()
        else :
            print('reset to 0')
            self.idx = 0
            self.current_goal.x = self.via_point[0][self.idx]
            self.current_goal.y = self.via_point[1][self.idx]
            self.send_set_goal_request(self.current_goal)
            self.send_enable_request()
        return response
    def rand_goal_callback(self,request,response):
        response.position = Point()
        self.goal = 9 * np.random.rand(2) + 0.5
        response.position.x = self.goal[0]
        response.position.y = self.goal[1]
        self.send_set_goal_request(response.position)
        print('25')
        self.send_enable_request()
        
        self.get_logger().info(f'New goal => x:{self.goal[0]},y:{self.goal[1]}')    
        return response
        
    def send_enable_request(self):    
        req = Empty.Request()
        self.future = self.enable_client.call_async(req)
        
    def send_set_goal_request(self,position):    
        req = SetGoal.Request()
        req.position = position
        self.future = self.set_goal_client.call_async(req)
        

def main(args=None):
    rclpy.init(args=args)
    scheduler = Scheduler()
    rclpy.spin(scheduler)
    scheduler.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
