#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import requests
from rclpy.parameter import Parameter
import sys

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        
        self.declare_parameter('server_ip', '127.0.0.1')
        self.declare_parameter('stream_on', False)
        
        self.server_ip = self.get_parameter('server_ip').value
        self.stream_on = self.get_parameter('stream_on').value
        self.server_url = f"http://{self.server_ip}:5000/stream"
        
        self.publisher_ = self.create_publisher(Image, 'cam_stream', 10)
        self.subscription = self.create_subscription(
            Image,
            'cam_stream',
            self.viewer_callback,
            qos_profile_system_default)
        
        self.cap = cv2.VideoCapture(0)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        
        self.br = CvBridge()
        self.timer = self.create_timer(0.016, self.timer_callback) 
        
        self.get_logger().info(f'Stream enabled: {self.stream_on}')
        self.get_logger().info(f'Server IP: {self.server_ip}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Upscale image if resolution is lower than desired
            if frame.shape[1] != 1920 or frame.shape[0] != 1080:
                frame = cv2.resize(frame, (1920, 1080), interpolation=cv2.INTER_CUBIC)
            
            # Publish the frame
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            
            if self.stream_on:
                try:
                    _, buffer = cv2.imencode('.jpg', frame)
                    requests.post(
                        self.server_url, 
                        files={"frame": buffer.tobytes()},
                        timeout=1
                    )
                except requests.exceptions.RequestException:
                    self.get_logger().warning('Failed to stream to server')

    def viewer_callback(self, data):
        # No longer needed for display, removing the function
        pass

    def __del__(self):
        self.cap.release()

def main():
    rclpy.init()
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
