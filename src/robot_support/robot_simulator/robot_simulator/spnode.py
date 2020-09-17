import json

from rclpy.node import Node
from sp_messages.msg import RegisterResource
from sp_messages.msg import Resources

class SPNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Resource
        self.resource = RegisterResource()
        self.resource.path = self.get_namespace()
        self.resource.model = ""
        self.resource.last_goal_from_sp = ""

        self.sp_node_cmd_subscriber = self.create_subscription(
            Resources,
            "/sp/resources",
            self.sp_resources_callback,
            10)

        self.sp_resource_publisher = self.create_publisher(
            RegisterResource,
            "/sp/resource",
            10)

    def sp_resources_callback(self, data):
        if not self.resource.path in data.resources:
            self.get_logger().info('The resource ' + str(self.resource.path) + ' is not registered in :' + str(data.resources))
            self.sp_resource_publisher.publish(self.resource)

    def goal_to_json(self, msg_type, goal):
         # move to general function in sp
        goal_to_json = {}
        for k in msg_type.get_fields_and_field_types().keys():
            goal_to_json.update({k: getattr(goal, "_"+k)})

        self.resource.last_goal_from_sp = json.dumps(goal_to_json)

    def has_last_goal(self):
        return bool(self.resource.last_goal_from_sp)
