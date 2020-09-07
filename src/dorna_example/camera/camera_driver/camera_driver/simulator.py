import sys
import rclpy
import time
import csv
import os
import math
import numpy
import ast
import json
import yaml
import random
from rclpy.node import Node

from camera_msgs.msg import Goal
from camera_msgs.msg import Measured

from sp_messages.msg import NodeCmd
from sp_messages.msg import NodeMode
from sp_messages.msg import RegisterResource
from ament_index_python.packages import get_package_share_directory

class CameraSimulator(Node):

    def __init__(self):
        super().__init__("camera_simulator")

        # initial state
        self.scanning = False
        self.done = False
        self.result = 0

        # remember last goal
        self.last_seen_goal = Goal()
        self.last_seen_goal.do_scan = False

        # sp node mode
        self.sp_node_cmd = NodeCmd()
        self.mode = NodeMode()
        self.mode.mode = "init"

        # Resource
        self.resource = RegisterResource()
        self.resource.path = "sp/camera/" + self.get_name()  # get name from node
        self.resource.model = ""
        self.resource.last_goal_from_sp = ""

        self.subscriber = self.create_subscription(
            Goal,
            "goal",
            self.sp_goal_callback,
            10)

        self.sp_node_cmd_subscriber = self.create_subscription(
            NodeCmd,
            "node_cmd",
            self.sp_node_cmd_callback,
            10)

        self.state_publisher = self.create_publisher(
            Measured,
            "measured",
            10)

        self.sp_mode_publisher = self.create_publisher(
            NodeMode,
            "mode",
            10)

        self.sp_resource_publisher = self.create_publisher(
            RegisterResource,
            "/sp/resource",
            10)

        self.timer = self.create_timer(2, self.tick)
        self.get_logger().info('Camera up and running!')

    def publish_state(self):
        msg = Measured()
        msg.scanning = self.scanning
        msg.done = self.done
        msg.result = self.result
        self.state_publisher.publish(msg)

    def tick(self):
        # tick our state machine
        if self.scanning and not self.done:
            self.result = random.randint(1, 3)
            self.get_logger().info('Camera scan returned %s' % self.result)
            self.done = True

        self.publish_state()
        self.sp_resource_publisher.publish(self.resource)


    def sp_goal_callback(self, data):
        if data.do_scan and not self.done:
            if self.scanning == False:
                self.get_logger().info("Starting camera scan")

            self.scanning = True
            self.done = False
        elif not data.do_scan:
            self.scanning = False
            self.done = False
            self.result = 0

        self.last_seen_goal = data
        self.publish_state()

    def sp_node_cmd_callback(self, data):
        self.sp_node_cmd = data

        # move to general function in sp
        echo_msg = {}
        for k in Goal.get_fields_and_field_types().keys():
            echo_msg.update({k: getattr(self.last_seen_goal, "_"+k)})

        self.mode.echo = json.dumps(echo_msg)

        if self.sp_node_cmd.mode == "run":
            self.mode.mode = "running"
        else:
            self.mode.mode = "init"

        self.sp_mode_publisher.publish(self.mode)


def main(args=None):

    rclpy.init(args=args)
    node = CameraSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
