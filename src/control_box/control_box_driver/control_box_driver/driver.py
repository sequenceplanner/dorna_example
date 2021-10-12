
import rclpy
import pyfirmata
from rclpy.node import Node
from .spnode import SPNode

from control_box_msgs.msg import Goal
from control_box_msgs.msg import Measured

from ament_index_python.packages import get_package_share_directory

class ControlBoxDriver(SPNode):

    def __init__(self):
        super().__init__("control_box_driver")

        self.board = pyfirmata.Arduino('/dev/ttyARDUINO')

        # internal state
        self.blue_light = False

        # initial state off
        self.board.digital[3].write(self.blue_light == False)


        self.subscriber = self.create_subscription(
            Goal,
            "goal",
            self.sp_goal_callback,
            10)

        self.state_publisher = self.create_publisher(
            Measured,
            "measured",
            10)

        self.timer = self.create_timer(2, self.pub_state)
        self.get_logger().info('Control box up and running!')

    def sp_goal_callback(self, data):
        if self.has_last_goal() and self.blue_light == data.blue_light:
            return

        self.blue_light = data.blue_light

        # update light
        self.board.digital[3].write(self.blue_light == False)
        self.get_logger().info("blue light is " + ('on' if self.blue_light else 'off'))

        self.goal_to_json(Goal, data)
        self.pub_state()

    def pub_state(self):
        msg = Measured()
        msg.blue_light_on = self.blue_light
        self.state_publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = ControlBoxDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
