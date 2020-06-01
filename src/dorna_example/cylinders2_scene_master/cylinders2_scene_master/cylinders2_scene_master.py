import sys
import os
import rclpy
import time
import json

from builtin_interfaces.msg import Time

from rclpy.node import Node
from sp_messages.msg import RunnerInfo
from sp_messages.msg import State
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from ros2_scene_manipulation_msgs.srv import ManipulateScene

class SceneMaster(Node):

    def __init__(self):
        super().__init__("cylinders2_scene_master")

        self.manipulate_scene_client = self.create_client(ManipulateScene, 'scene_manipulation_service')

        while not self.manipulate_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ManipulateScene.Request()

        self.callback_timeout = time.time()
        self.timeout = 5

        self.marker_timer_period = 0.03

        self.sp_path_to_product_name = {
            'cylinders2/dorna_holding': 'dorna',
            'cylinders2/dorna3_holding': 'dorna3',
            'cylinders2/shelf1': 'shelf1',
            'cylinders2/shelf2': 'shelf2',
            'cylinders2/shelf3': 'shelf3',
            'cylinders2/conveyor': 'conveyor',
            'cylinders2/conveyor2': 'conveyor2',
        }

        self.products = {
            'dorna' : 0,
            'dorna3' : 0,
            'shelf1' : 0,
            'shelf2' : 0,
            'shelf3': 0,
            'conveyor' : 0,
            'conveyor2': 0,
        }

        self.product_to_frame = {
            1: 'cylinders2/c1/cube',
            2: 'cylinders2/c2/cube',
            3: 'cylinders2/c3/cube',
        }

        red = ColorRGBA()
        red.a = 1.0
        red.r = 1.0
        green = ColorRGBA()
        green.a = 1.0
        green.g = 1.0
        blue = ColorRGBA()
        blue.a = 1.0
        blue.b = 1.0
        self.product_colors = {
            1: red,
            2: green,
            3: blue,
        }

        self.slot_to_frame = {
            'dorna' : 'dorna/r1/dorna_5_link',
            'dorna3' : 'dorna/r3/dorna_5_link',
            'shelf1' : '/cylinders2/shelf1',
            'shelf2' : '/cylinders2/shelf2',
            'shelf3' : '/cylinders2/shelf3',
            'conveyor' : '/cylinders2/conveyor',
            'conveyor2': '/cylinders2/conveyor2',
        }

        self.product_marker_publishers = {
            1: self.create_publisher(Marker, "cylinders2/sm/cube1_marker", 20),
            2: self.create_publisher(Marker, "cylinders2/sm/cube2_marker", 20),
            3: self.create_publisher(Marker, "cylinders2/sm/cube3_marker", 20),
        }

        self.sm_sp_runner_subscriber = self.create_subscription(
            RunnerInfo,
            "sp/runner/info",
            self.sp_runner_callback,
            20)

        self.marker_timer = self.create_timer(
            self.marker_timer_period,
            self.publish_markers)

    def cube_marker(self, frame, color):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = Time()
        marker.ns = ""
        marker.id = 0
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color = color

        return marker

    def publish_markers(self):
        for key, frame in self.product_to_frame.items():
            color = self.product_colors[key]
            marker = self.cube_marker(frame, color)
            self.product_marker_publishers[key].publish(marker)

    def sp_runner_callback(self, data):
        old = set(self.products.items())
        for s in data.state:
            pn = self.sp_path_to_product_name.get(s.path)
            if pn != None:
                v = json.loads(s.value_as_json)
                self.products[pn] = v

        # update what has changed
        new = set(self.products.items())
        changes = new - old
        if changes != set():
            print(changes)
            for slot, prod in changes:
                self.handle_change(slot, prod)

        self.remove_empty()


    def handle_change(self, slot, prod):
        for key, value in self.products.items():
            if value == prod:
                self.products[key] = 0  # "move" the item (it can only be in one slot)
        self.products[slot] = prod
        # we never "detach" an item, just attach it to its new owner. (with teleport)
        if prod != 0:
            child = self.product_to_frame[prod]
            parent = self.slot_to_frame[slot]
            self.send_request(child, parent, False)

    def remove_empty(self):
        for p, pv in self.product_to_frame.items():
            used = False
            for slot, value in self.products.items():
                if p == value:
                    used = True

            if not used:
                # product not in use, move away (to store)
                self.send_request(pv, "/cylinders2/product_store", False)



    def send_request(self, frame, parent, pos):
        self.req.frame_id = frame
        self.req.parent_id = parent
        self.req.transform.translation.x = 0.0
        self.req.transform.translation.y = 0.0
        self.req.transform.translation.z = 0.0
        self.req.transform.rotation.x = 0.0
        self.req.transform.rotation.y = 0.0
        self.req.transform.rotation.z = 0.0
        self.req.transform.rotation.w = 1.0
        self.req.same_position_in_world = pos
        self.future = self.manipulate_scene_client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    sm = SceneMaster()
    rclpy.spin(sm)
    sm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
