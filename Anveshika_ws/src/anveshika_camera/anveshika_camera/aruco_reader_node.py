import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from cv2 import aruco
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

class ArucoReaderNode(Node):
    def __init__(self):
        super().__init__('aruco_reader_node')
        self.get_logger().info('ArUco Reader Node Initialized')

        self.br = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'cam_stream',
            self.listener_callback,
            10)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.polygonalApproxAccuracyRate = 0.05

        self.x_pub = self.create_publisher(Float32, 'aruco_reader/x', 10)
        self.y_pub = self.create_publisher(Float32, 'aruco_reader/y', 10)
        self.id_pub = self.create_publisher(Float32, 'aruco_reader/id', 10)

    def listener_callback(self, data):
        frame = self.br.imgmsg_to_cv2(data)

        frame_height, frame_width = frame.shape[:2]
        screen_center = (frame_width // 2, frame_height // 2)

        cv2.circle(frame, screen_center, 5, (0, 255, 0), -1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids):
                pts = marker_corners[0].astype(np.int32)
                marker_center = (
                    int((pts[0][0] + pts[2][0]) / 2),
                    int((pts[0][1] + pts[2][1]) / 2)
                )

                distance_x = float(marker_center[0] - screen_center[0])
                distance_y = float(marker_center[1] - screen_center[1])

                self.get_logger().info(f"Marker ID: {marker_id[0]}, Position: ({distance_x}, {distance_y})")

                frame = aruco.drawDetectedMarkers(frame, corners, ids)

                cv2.circle(frame, marker_center, 5, (0, 0, 255), -1)

                distance_text = f"Dist: ({distance_x:.2f}, {distance_y:.2f})"
                cv2.putText(frame, distance_text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

                if marker_id[0] == 42:
                    x_msg = Float32()
                    x_msg.data = float(distance_x)
                    self.x_pub.publish(x_msg)

                    y_msg = Float32()
                    y_msg.data = float(distance_y)
                    self.y_pub.publish(y_msg)

                    id_msg = Float32()
                    id_msg.data = float(marker_id[0])
                    self.id_pub.publish(id_msg)

        cv2.imshow("Camera Feed with ArUco", frame)
        cv2.waitKey(1)

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
