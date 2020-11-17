
import random
import rclpy
import json

from .spnode import SPNode
from rclpy.node import Node

from std_msgs.msg import String
from camera_msgs.msg import Goal
from camera_msgs.msg import Measured

from ament_index_python.packages import get_package_share_directory

class CameraSimulator(SPNode):

    def __init__(self):
        super().__init__("camera_simulator")

        # initial state
        self.scanning = False
        self.done = False
        self.result = 0

        # initial goal
        self.goal_to_json(Goal, Goal(do_scan = False))

        self.subscriber = self.create_subscription(
            Goal,
            "goal",
            self.sp_goal_callback,
            10)

        self.simulator_state = self.create_subscription(
            String,
            "/simulator_state",
            self.simulator_callback,
            10)

        self.state_publisher = self.create_publisher(
            Measured,
            "measured",
            10)

        self.timer = self.create_timer(2, self.tick)

        self.get_logger().info('Camera up and running!')

        g = Goal()
        self.sp_goal_callback(g)
        self.cubes = []

    def publish_state(self):
        msg = Measured()
        msg.scanning = self.scanning
        msg.done = self.done
        msg.result = self.result
        self.state_publisher.publish(msg)

    def tick(self):
        if self.scanning and not self.done:
            for cube in self.cubes:
                if cube["position"] == '/dorna/r1/dorna_5_link' and cube['revealed_color'] != 0:
                    self.done = True
                    self.result = cube['revealed_color']

        self.publish_state()


    def simulator_callback(self, msg):
        # data sload json
        # get color value for cube with position "camera".
        self.cubes = json.loads(msg.data)

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

        self.goal_to_json(Goal, data)
        self.publish_state()







def main(args=None):

    rclpy.init(args=args)
    node = CameraSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
