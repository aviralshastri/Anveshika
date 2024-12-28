import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import Int16

class ModesNode(Node):
    def __init__(self):
        super().__init__('modes_node')
        self.mode_publisher_ = self.create_publisher(Int16, 'mode_read', qos_profile_system_default)
        self.mode_subscriber_ = self.create_subscription(Int16, 'mode_change', self.subscriber_callback, qos_profile_system_default)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Modes list
        # -1 - Stop
        # 0  - Rotate to Coordinate
        # 1  - Move to Coordinate
        # 2  - Dig
        # 3  - Dump
        # 4  - Rotate to Aruco Marker
        # 5  - Move to Aruco Marker

        # Order to Follow
        # -1 - 0 - 1 - 2 - 0 - 1 - 4 - 5 - 3 - -1
        self.modes = [-1, 0, 1, 2, 0, 1, 4, 5, 3, -1]
        self.i = 0

    def subscriber_callback(self, msg):
        self.i += 1
        
    def timer_callback(self):
        msg = Int16()
        msg.data = self.modes[self.i]
        self.mode_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    modes_node = ModesNode()

    rclpy.spin(modes_node)

    modes_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()