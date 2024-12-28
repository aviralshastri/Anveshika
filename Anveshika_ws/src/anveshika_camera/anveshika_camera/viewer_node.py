#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.subscription = self.create_subscription(
            Image,
            'cam_stream',
            self.listener_callback,
            10)
        self.br = CvBridge()
        
    def listener_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)
        
    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()
    rclpy.spin(camera_viewer)
    camera_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()