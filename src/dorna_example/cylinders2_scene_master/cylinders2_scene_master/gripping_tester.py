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
from gripper_msgs.msg import Goal as GripperGoal

class Tester(Node):

    def __init__(self):
        super().__init__("gripping_tester")

        self.poses = ["zero", "after_homing", "test_pose", "scan", "take1", "take2", "take3", "pre_take", "leave"]

        self.callback_timeout = time.time()
        self.gripper_publisher_timer_period = 1
        self.publisher_timer_period = 0.001
        self.sp_publisher_timer_period = 0.5
        self.t_period = 7
        self.timeout = 5

        self.r1_robot_publisher = self.create_publisher(RobotGoal, "/dorna/r1/goal", 20)
        self.gripper_publisher = self.create_publisher(GripperGoal, "/gripper/goal", 20)
        self.sp_publisher = self.create_publisher(String, "/simulator_command", 20)

        self.r1_publisher_timer = self.create_timer(
            self.publisher_timer_period,
            self.r1_robot_publisher_callback)

        self.gripper_publisher_timer = self.create_timer(
            self.gripper_publisher_timer_period,
            self.gripper_robot_publisher_callback)
            
        self.sp_publisher_timer = self.create_timer(
            self.sp_publisher_timer_period,
            self.sp_publisher_callback)

        self.r1_robot_goal = "unknown"
        self.gripper_goal = False

        self.command = {
            "make_cube" : False,
            "remove_cube" : False,
            "scanned" : False 
        }

        self.pose_changer()

    def pose_changer(self):
        def pose_changer_local():
            while(1):

                time.sleep(4)
                self.r1_robot_goal = "leave"

                time.sleep(4)
                self.command = {
                    "make_cube" : True,
                    "remove_cube" : False,
                    "scanned" : False 
                }

                time.sleep(4)
                self.gripper_goal = True

                time.sleep(4)
                self.r1_robot_goal = "after_homing"

                time.sleep(4)
                self.command = {
                    "make_cube" : False,
                    "remove_cube" : True,
                    "scanned" : False 
                }

        t = threading.Thread(target=pose_changer_local)
        t.daemon = True
        t.start()

    def sp_publisher_callback(self):
        x = String()
        x.data = json.dumps(self.command)
        self.sp_publisher.publish(x)
        
    def r1_robot_publisher_callback(self):
        msg = RobotGoal()
        msg.ref_pos = self.r1_robot_goal
        self.r1_robot_publisher.publish(msg)
    
    def gripper_robot_publisher_callback(self):
        msg = GripperGoal()
        msg.close = self.gripper_goal
        self.gripper_publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    t = Tester()
    rclpy.spin(t)
    t.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
