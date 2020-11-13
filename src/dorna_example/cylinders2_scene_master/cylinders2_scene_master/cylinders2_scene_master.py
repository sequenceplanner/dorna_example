import sys
import os
import rclpy
import time
import json
import math
import random

from builtin_interfaces.msg import Time

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from ros2_scene_manipulation_msgs.srv import ManipulateScene

def euler_to_quaternion(yaw, pitch, roll):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

# class Command():
#     make_cube = False
#     remove_cube = False
#     scanned = False
#     gripper_close = False
#     gripper_open = False

# class CubeState():
#     cube_id: 0
#     true_color: ColorRGBA()
#     revealed_color: ColorRGBA()
#     position: "Unknown"

class SceneMaster(Node):

    def __init__(self):
        super().__init__("cylinders2_scene_master")
        # Command.__init__(self)
        # CubeState.__init__(self)

        self.manipulate_scene_client = self.create_client(ManipulateScene, 'scene_manipulation_service')

        while not self.manipulate_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ManipulateScene.Request()

        self.callback_timeout = time.time()
        self.timeout = 5

        self.marker_timer_period = 0.03
        self.sp_publisher_timer_period = 0.1

        self.sp_path_to_product_name = {
            '/cylinders2/product_state/dorna_holding': 'dorna',
            '/cylinders2/product_state/dorna3_holding': 'dorna3',
            '/cylinders2/product_state/shelf1': 'shelf1',
            '/cylinders2/product_state/shelf2': 'shelf2',
            '/cylinders2/product_state/shelf3': 'shelf3',
            '/cylinders2/product_state/conveyor': 'conveyor',
            '/cylinders2/product_state/conveyor2': 'conveyor2',
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

        self.product_types = {
            1: 100,
            2: 100,
            3: 100,
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
        white = ColorRGBA()
        white.a = 1.0
        white.r = 1.0
        white.g = 1.0
        white.b = 1.0
        self.product_colors = {
            0: white,
            1: red,
            2: green,
            3: blue
        }

        self.colors = [1, 2, 3]

        self.slot_to_frame = {
            'dorna' : 'dorna/r1/dorna_5_link',
            'dorna3' : 'dorna/r3/dorna_5_link',
            'shelf1' : '/cylinders2/shelf1',
            'shelf2' : '/cylinders2/shelf2',
            'shelf3' : '/cylinders2/shelf3',
            'conveyor' : '/cylinders2/conveyor',
            'conveyor2': '/cylinders2/conveyor2',
        }

        self.product_marker_publishers = {}

        self.marker_timer = self.create_timer(
            self.marker_timer_period,
            self.marker_publisher_callback)

        self.sp_subscriber = self.create_subscription(
            String,
            "/simulator_command",
            self.sp_runner_callback,
            20)

        self.sp_publisher = self.create_publisher(String, "/simulator_state", 20)

        self.sp_publisher_timer = self.create_timer(
            self.sp_publisher_timer_period,
            self.sp_publisher_callback)

        self.get_logger().info(str(self.get_name()) + ' is up and running')

        self.living_cubes = []

    def sp_runner_callback(self, msg):
        Command = {
            "make_cube" : False,
            "remove_cube" : False,
            "scanned" : False 
        }

        try:
            Command = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.get_logger().error('error in sp_runner_callback: "%s"' % error)
        
        self.get_logger().info('got Command: "%s"' % Command)
        if Command["make_cube"]:
            if not any((cube["position"] == '/cylinders2/conveyor2') for cube in self.living_cubes):
                self.living_cubes.append(self.make_cube())
            else:
                self.get_logger().info('OCCUPIED CUBE ADDING SPACE')
        
        if Command["remove_cube"]:
            self.remove_cube()

        if Command["scanned"]:
            self.update_color()

    def make_cube(self):
        
        self.living_cubes_ids = []
        for lc in self.living_cubes:
            self.living_cubes_ids.append(lc["cube_id"])
        dif = self.difference(range(1, 4), self.living_cubes_ids)
        self.get_logger().info('difference: ' + str(dif))

        cube = {
            "cube_id" : 0,
            "true_color" : 0,
            "revealed_color" : 0,
            "position" : "Unknkown"
        }

        # cube = Cube
        cube["cube_id"] = random.choice(dif) # for now, fix later, need choice from diff
        self.living_cubes.append(cube)
        cube["true_color"] = random.choice(self.colors)
        cube["revealed_color"] = 0
        cube["position"] = '/cylinders2/conveyor2'

        self.product_marker_publishers[cube["cube_id"]] = self.create_publisher(Marker, "cylinders2/sm/cube" + str(cube["cube_id"]) + "_marker", 20)
        self.get_logger().info('ADDING CUBE: ' + str(cube["cube_id"]))

        return cube
        
    def remove_cube(self):
        for cube in self.living_cubes:
            if cube["position"] == '/cylinders2/conveyor2':
                self.living_cubes.remove(cube)
                # marker = self.make_or_remove_marker(cube, 2)
                # self.product_marker_publishers[cube["cube_id"]].publish(marker) 
                self.remove_key(self.product_marker_publishers, cube["cube_id"])
                self.get_logger().info('REMOVING CUBE: ' + str(cube["cube_id"]))
    
    def remove_key(self, d, key):
        r = dict(d)
        del r[key]
        return r

    def difference(self, list1, list2):
        return (list(list(set(list1)-set(list2)) + list(set(list2)-set(list1))))

    # def change parrent:
    #     listen to gripper topic


    # def update_color(self):
        

    def sp_publisher_callback(self):
        x = String()
        x.data = json.dumps(self.living_cubes)
        self.sp_publisher.publish(x)
    
    def make_marker(self, cube):
        marker = Marker()
        marker.header.frame_id = cube["position"]
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
        marker.color = self.product_colors[cube["revealed_color"]]

        return marker

    def marker_publisher_callback(self):
        for cube in self.living_cubes:
            marker = self.make_marker(cube)
            self.product_marker_publishers[cube["cube_id"]].publish(marker)   

    def remove_marker(self):
        for cube in self.living_cubes:
            if cube["position"] == '/cylinders2/conveyor2':
                self.send_request("cylinders2/sm/cube" + str(cube["cube_id"], "/cylinders2/product_store", False))

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
