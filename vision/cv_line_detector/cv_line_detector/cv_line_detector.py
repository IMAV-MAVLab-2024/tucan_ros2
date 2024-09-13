#! /usr/bin/env python3
# OPENCV
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from tucan_msgs.msg import LineFollower

# 相机校准参数
camera_matrix = np.array([[528.673438813997, 0, 362.958493066534],
                          [0, 569.793218233108, 283.723935140803],
                          [0, 0, 1]])
dist_coeffs = np.array([0.138739907567143, -0.272661915942306, 0, 0, 0])

# 相机到机体坐标系的旋转矩阵和位移向量
R_frd_cam = np.array([[0, -1,  0],  # 从相机坐标系到机体坐标系的旋转矩阵
                      [1,  0,  0],
                      [0,  0,  1]], dtype=np.float32)
t_frd_cam = np.array([0.06, 0.0, 0.0], dtype=np.float32)  # 位移向量

class LineDetector(Node):
    def __init__(self):
        super().__init__("cv_line_detector")
        self.get_logger().info("CV Line Detection Node has been started")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers and publishers
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.line_detection_publisher = self.create_publisher(LineFollower, "/cv_line_detection", qos_profile)
        self.bridge_for_CV = CvBridge()
        self.subscription = self.create_subscription(Image, "/down_camera_image", self.ImageLoop, qos_profile)

        # 记录之前的状态
        self.previous_x = 0
        self.previous_y = 0
        self.vehicle_odometry = None

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = msg

    def ImageLoop(self, data):
        img = self.bridge_for_CV.imgmsg_to_cv2(data)
        blue_lower = np.array([100, 50, 50])
        blue_upper = np.array([130, 255, 255])
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_img, blue_lower, blue_upper)
        edges = cv2.Canny(blue_mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)

        if lines is not None and self.vehicle_odometry is not None:
            line_centers = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                line_centers.append((center_x, center_y))

            if len(line_centers) > 0:
                avg_x, avg_y = np.mean(line_centers, axis=0)

                # 计算目标点在相机坐标系中的位置
                # 这里我们假设z_cam为0
                x_cam = (avg_x - camera_matrix[0, 2]) / camera_matrix[0, 0]
                y_cam = (avg_y - camera_matrix[1, 2]) / camera_matrix[1, 1]
                z_cam = 0

                # 从相机坐标系转换到机体坐标系
                point_cam = np.array([x_cam, y_cam, z_cam])
                point_frd = np.dot(R_frd_cam, point_cam) + t_frd_cam

                # 从机体坐标系转换到NED坐标系
                orientation = self.vehicle_odometry.q
                R_body_to_ned = R.from_quat([orientation[1], orientation[2], orientation[3], orientation[0]]).as_matrix()
                point_ned = np.matmul(R_body_to_ned, point_frd) + np.array([self.vehicle_odometry.position[0], 
                                                                           self.vehicle_odometry.position[1], 
                                                                           self.vehicle_odometry.position[2]])

                msg = LineFollower()
                msg.x = float(point_ned[0])
                msg.y = float(point_ned[1])
                msg.z = float(point_ned[2])
                self.line_detection_publisher.publish(msg)

                self.get_logger().debug(f"Publishing: X: {msg.x}, Y: {msg.y}, Z: {msg.z}")

        else:
            msg = LineFollower()
            msg.x = float(self.previous_x)
            msg.y = float(self.previous_y)
            msg.z = 0.0  # Default value
            self.line_detection_publisher.publish(msg)
            self.get_logger().debug(f"Publishing: X: {msg.x}, Y: {msg.y}, Z: {msg.z}")

def main():
    rclpy.init()
    line_detector_node = LineDetector()
    try:
        rclpy.spin(line_detector_node)
    except KeyboardInterrupt:
        print("Shutting Down")
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
