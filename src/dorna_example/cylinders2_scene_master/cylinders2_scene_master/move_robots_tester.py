import os
import rclpy
import time
import json
import math
import threading

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from ros2_scene_manipulation_msgs.srv import ManipulateScene

from robot_msgs.msg import RobotState, RobotGoal

class Tester(Node):

    def __init__(self):
        super().__init__("move_robots_tester")

        self.poses = ["zero", "after_homing", "test_pose", "scan", "take1", "take2", "take3", "pre_take", "leave"]

        self.callback_timeout = time.time()
        self.publisher_timer_period = 0.001
        self.t_period = 7
        self.timeout = 5

        self.r1_robot_publisher = self.create_publisher(RobotGoal, "/dorna/r1/goal", 20)
        self.r2_robot_publisher = self.create_publisher(RobotGoal, "/dorna/r2/goal", 20)
        self.r3_robot_publisher = self.create_publisher(RobotGoal, "/dorna/r3/goal", 20)
        self.r4_robot_publisher = self.create_publisher(RobotGoal, "/dorna/r4/goal", 20)

        self.r1_publisher_timer = self.create_timer(
            self.publisher_timer_period,
            self.r1_robot_publisher_callback)

        self.r1_publisher_timer = self.create_timer(
            self.publisher_timer_period,
            self.r2_robot_publisher_callback)
        
        self.r3_publisher_timer = self.create_timer(
            self.publisher_timer_period,
            self.r3_robot_publisher_callback)

        self.r4_publisher_timer = self.create_timer(
            self.publisher_timer_period,
            self.r4_robot_publisher_callback)
        
        self.r1_robot_goal = "unknown"
        self.r2_robot_goal = "unknown"
        self.r3_robot_goal = "unknown"
        self.r4_robot_goal = "unknown"

        self.pose_changer()

    def pose_changer(self):
        def pose_changer_local():
            while(1):
                for pose in self.poses:
                    print(pose)
                    self.r1_robot_goal = pose
                    self.r2_robot_goal = pose
                    self.r3_robot_goal = pose
                    self.r4_robot_goal = pose
                    time.sleep(4)
        t = threading.Thread(target=pose_changer_local)
        t.daemon = True
        t.start()
        
    def r1_robot_publisher_callback(self):
        msg = RobotGoal()
        msg.ref_pos = self.r1_robot_goal
        self.r1_robot_publisher.publish(msg)
    
    def r2_robot_publisher_callback(self):
        msg = RobotGoal()
        msg.ref_pos = self.r2_robot_goal
        self.r2_robot_publisher.publish(msg)
        
    def r3_robot_publisher_callback(self):
        msg = RobotGoal()
        msg.ref_pos = self.r3_robot_goal
        self.r3_robot_publisher.publish(msg)

    def r4_robot_publisher_callback(self):
        msg = RobotGoal()
        msg.ref_pos = self.r4_robot_goal
        self.r4_robot_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    t = Tester()
    rclpy.spin(t)
    t.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
