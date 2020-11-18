import os
import rclpy
import time
import json
import math

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from ros2_scene_manipulation_msgs.srv import ManipulateScene

class Tester(Node):

    def __init__(self):
        super().__init__("add_remove_cube_tester")

        self.callback_timeout = time.time()
        self.sp_publisher_timer_period = 5
        self.timeout = 5

        self.sp_publisher = self.create_publisher(String, "/simulator_command", 20)

        self.sp_publisher_timer = self.create_timer(
            self.sp_publisher_timer_period,
            self.sp_publisher_callback)

    def sp_publisher_callback(self):

        Command = {
            "make_cube" : True,
            "remove_cube" : False,
            "scanned" : False 
        }

        x = String()
        x.data = json.dumps(Command)
        self.sp_publisher.publish(x)

        time.sleep(2)

        Command = {
            "make_cube" : False,
            "remove_cube" : True,
            "scanned" : False 
        }

        x = String()
        x.data = json.dumps(Command)
        self.sp_publisher.publish(x)

        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)

    t = Tester()
    rclpy.spin(t)
    t.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()