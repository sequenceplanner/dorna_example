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

from gripper_msgs.msg import Goal as GripperGoal, State as GripperState
from camera_msgs.msg import Goal as CameraGoal, Measured as CameraState
from robot_msgs.msg import RobotState

def euler_to_quaternion(yaw, pitch, roll):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

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

        self.slot_to_frame = {
            'dorna' : 'dorna/r1/dorna_5_link',
            'dorna3' : 'dorna/r3/dorna_5_link',
            'shelf1' : '/cylinders2/shelf1',
            'shelf2' : '/cylinders2/shelf2',
            'shelf3' : '/cylinders2/shelf3',
            'conveyor' : '/cylinders2/conveyor',
            'conveyor2': '/cylinders2/conveyor2',
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

        # idea is to add and remove them on the fly
        self.product_marker_publishers = {}

        self.marker_timer = self.create_timer(
            self.marker_timer_period,
            self.marker_publisher_callback)

        self.r1_robot_position = "unknown"
        self.r1_robot_subscriber = self.create_subscription(
            RobotState,
            "/dorna/r1/measured",
            self.r1_robot_callback,
            20)

        self.r3_robot_position = "unknown"
        self.r3_robot_subscriber = self.create_subscription(
            RobotState,
            "/dorna/r3/measured",
            self.r3_robot_callback,
            20)
        
        self.gripper_goal = False
        self.gripper_subscriber = self.create_subscription(
            GripperGoal,
            "/gripper/goal",
            self.gripper_callback,
            20)

        self.camera_scanning_true = False
        self.camera_done = False
        self.camera_subscriber = self.create_subscription(
            CameraState,
            "/camera/state",
            self.camera_callback,
            20)

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

    def r1_robot_callback(self, msg):
        self.r1_robot_position = msg.act_pos

    def r3_robot_callback(self, msg):
        self.r3_robot_position = msg.act_pos
        for cube in self.living_cubes:
            if cube['position'] == "/cylinders2/conveyor2" and self.r3_robot_position == "pre_take":
                self.cube_to_parent(cube['cube_id'], "/dorna/r3/dorna_5_link")
            if cube['position'] == "/dorna/r3/dorna_5_link" and self.r3_robot_position == "leave":
                self.cube_to_parent(cube['cube_id'], "/cylinders2/conveyor")

    def gripper_callback(self, msg):
        self.gripper_goal = msg.close
        # self.gripper_goal = msg.close
        # introduce possibility to fail grasping (1/5)
        fail_list = [True, True, True, True, False]
        for cube in self.living_cubes:
            # would be nice to have matching pose names, then: if cube['position'] == self.slot_to_frame[self.r1_robot_position]
            if cube['position'] == "/cylinders2/conveyor" and self.r1_robot_position == "leave" or \
                cube['position'] == "/cylinders2/shelf1" and self.r1_robot_position == "take1" or \
                cube['position'] == "/cylinders2/shelf2" and self.r1_robot_position == "take2" or \
                cube['position'] == "/cylinders2/shelf3" and self.r1_robot_position == "take3":
                if random.choice(fail_list):
                    self.cube_to_parent(cube['cube_id'], "/dorna/r1/dorna_5_link")
                    self.get_logger().info('GRASPING SUCCESS')
                else:
                    self.get_logger().info('GRASPING FAILED')
    
    def camera_callback(self, msg):
        self.camera_scanning_true = msg.scanning
        self.camera_done = msg.done
        # introduce possibility to fail scanning (1/5)
        fail_list = [True, True, True, True, False]
        for cube in self.living_cubes:
            if any((cube["position"] == '/dorna/r1/dorna_5_link') for cube in self.living_cubes):
                if self.r1_robot_position == "scan":
                    if self.camera_scanning_true and not self.camera_done:
                        if random.choice(fail_list):
                            cube['revealed_color'] = cube['actual_color']
                            self.get_logger().info('SCANNING SUCCESS')
                        else:
                            self.get_logger().info('SCANNING FAILED')

    def sp_runner_callback(self, msg):
        Command = {
            "make_cube" : False,
            "remove_cube" : False
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

            # remove duplicates because it is generated for some reason
            self.living_cubes = [dict(t) for t in {tuple(d.items()) for d in self.living_cubes}]
        
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

        cube["cube_id"] = random.choice(dif) # for now, fix later, need choice from diff
        self.living_cubes.append(cube)
        cube["true_color"] = random.choice(self.colors)
        cube["revealed_color"] = 0
        cube["position"] = '/cylinders2/conveyor2'

        self.product_marker_publishers[cube["cube_id"]] = self.create_publisher(Marker, "cylinders2/sm/cube" + str(cube["cube_id"]) + "_marker", 20)
        self.send_request("cylinders2/c" + str(cube["cube_id"]) +" /cube", "/cylinders2/conveyor2", False)
        self.get_logger().info('ADDING CUBE: ' + str(cube["cube_id"]))

        return cube
        
    def remove_cube(self):
        for cube in self.living_cubes:
            if cube["position"] == '/cylinders2/conveyor2':
                self.living_cubes.remove(cube)
                self.product_marker_publishers[cube["cube_id"]].publish(self.make_marker(cube))
                self.remove_key(self.product_marker_publishers, cube["cube_id"])
                self.send_request("cylinders2/c" + str(cube["cube_id"]) +" /cube", "/cylinders2/product_store", False)
                self.get_logger().info('REMOVING CUBE: ' + str(cube["cube_id"]))
    
    def remove_key(self, d, key):
        r = dict(d)
        del r[key]
        return r

    def cube_to_parent(self, cube_id, parent):
        for cube in self.living_cubes:
            if cube_id == cube["cube_id"]:
                cube["position"] = parent
                self.send_request("cylinders2/c" + str(cube["cube_id"]) +" /cube", parent, False)

    def difference(self, list1, list2):
        return (list(list(set(list1)-set(list2)) + list(set(list2)-set(list1))))

    def sp_publisher_callback(self):
        x = String()
        x.data = json.dumps(self.living_cubes)
        self.sp_publisher.publish(x)
    
    def make_marker(self, cube):
        marker = Marker()
        marker.header.frame_id = "cylinders2/c" + str(cube["cube_id"]) +" /cube"
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

    def camera_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = Time()
        marker.ns = ""
        marker.id = 0
        marker.type = 10
        marker.action = 0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.65

        quat = euler_to_quaternion(0.0, 0, 3.9)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]


        marker.scale.x = 0.00025
        marker.scale.y = 0.00075
        marker.scale.z = 0.00075

        c = ColorRGBA()
        c.a = 1.0
        c.r = 0.5
        c.g = 0.5
        c.b = 0.5

        marker.color = c
        marker.mesh_resource = "file://" + self.photoneo_mesh

        return marker

    def blue_light_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = Time()
        marker.ns = ""
        marker.id = 0
        marker.type = 3
        marker.action = 0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.2
        marker.pose.position.z = 0.5
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = 0.0
        # marker.pose.orientation.w = 1.0

        quat = euler_to_quaternion(0, 1.5707, 0)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]


        marker.scale.x = 0.075
        marker.scale.y = 0.075
        marker.scale.z = 0.01

        c = ColorRGBA()
        c.a = 1.0
        c.r = 0.2
        c.g = 0.2
        c.b = 0.5
        if self.blue_light_on:
            c.b = 1.0

        marker.color = c

        return marker

def main(args=None):
    rclpy.init(args=args)

    sm = SceneMaster()
    rclpy.spin(sm)
    sm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
