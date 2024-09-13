#! /usr/bin/env python3
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleStatus
from tucan_msgs.msg import LineFollower, Mode
from scipy.spatial.transform import Rotation as R
import math

camera_matrix = np.array([[528.673438813997, 0, 362.958493066534],
                          [0, 569.793218233108, 283.723935140803],
                          [0, 0, 1]])

dist_coeffs = np.array([0.138739907567143, -0.272661915942306, 0, 0, 0])

R_frd_cam = np.array([[0, -1,  0], # rotation matrix camera to front-right-down
                    [1,  0,  0],
                    [0,  0,  1]], dtype=np.float32)

t_frd_cam = np.array([-0.06, 0.0, 0.0], dtype=np.float32) # translation vector from camera to front-right-down

class LineTracker(Node):
    def __init__(self):
        super().__init__("cv_line_tracker")
        self.get_logger().info("CV Line Tracking Node has been started")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        self.line_detection_publisher = self.create_publisher(LineFollower, "/cv_line_detection", 1)
        self.bridge_for_CV = CvBridge()
        self.subscription = self.create_subscription(Image, "/down_camera_image", self.ImageLoop, 1)

        self.previous_x = 0
        self.previous_y = 0
        self.previous_yaw = 0
        self.previous_x_global = 0
        self.previous_y_global = 0
        self.previous_z_global = 0

        self.vehicle_odometry = None

        self.last_detection_timestamp = self.get_clock().now().to_msg()

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def ImageLoop(self, data):
        msg = LineFollower()
        img = self.bridge_for_CV.imgmsg_to_cv2(data, 'bgr8')

        # Convert image to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Define the color range for the blue color
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            # Fit a line to the largest contour
            [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)

            # Calculate line's angle
            yaw = np.arctan2(vy, vx)  # Angle in radians
           
            # Calculate the center of the detected line in the image
            center_x = np.mean([contours[:, 0, 0].min(), contours[:, 0, 0].max()])
            center_y = np.mean([contours[:, 0, 1].min(), contours[:, 0, 1].max()])

            # Convert the line center to NED frame
            if self.vehicle_odometry is not None:
                # Convert line center from FRD to NED

                # 估计目标点在相机坐标系下的位置
                Z_cam = abs(self.vehicle_odometry.position[2])  # 目标物体距离
                X_cam = (center_x - camera_matrix[0, 2]) / camera_matrix[0, 0] * Z_cam
                Y_cam = (center_y - camera_matrix[1, 2]) / camera_matrix[1, 1] * Z_cam

                # 相机坐标系到FRD坐标系的转换
                tvec_cam = np.array([X_cam, Y_cam, Z_cam], dtype=np.float32)
                tvec_frd = np.dot(R_frd_cam, tvec_cam) + t_frd_cam

                tvec_ned = np.matmul(R.from_quat([self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3], self.vehicle_odometry.q[0]]).as_matrix(), tvec_frd) + np.array([self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], self.vehicle_odometry.position[2]])

                pos_x = tvec_ned[0]
                pos_y = tvec_ned[1]
                pos_z = tvec_ned[2]


                self.last_detection_timestamp = self.get_clock().now().to_msg()
                msg.last_detection_timestamp = self.last_detection_timestamp

                msg.detected = True
                msg.x_global = float(pos_x)
                msg.y_global = float(pos_y)
                msg.z_global = float(pos_z)
                msg.x_picture = float(center_y)
                msg.y_picture = float(center_x)
                msg.yaw = float(yaw + self.quat_get_yaw(self.vehicle_odometry.q))

                self.previous_yaw = msg.yaw
                self.previous_x_global = pos_x
                self.previous_y_global = pos_y
                self.previous_z_global = pos_z
                self.previous_x = msg.x_picture
                self.previous_y = msg.y_picture

                self.line_detection_publisher.publish(msg)
                self.get_logger().debug("Publishing: X: %.2f Y: %.2f Z: %.2f relative_yaw: %.2f Yaw: %.2f Detected: True" % 
                                (msg.x_global, msg.y_global, msg.z_global, yaw, msg.yaw))
        else:
            msg.detected = False
            msg.x_global = float(self.previous_x_global)
            msg.y_global = float(self.previous_y_global)
            msg.z_global = float(self.previous_z_global)
            msg.yaw = float(self.previous_yaw)  # Default yaw if no line detected
            msg.x_picture = float(self.previous_y)
            msg.y_picture = float(self.previous_x)
            msg.last_detection_timestamp = self.last_detection_timestamp
            self.line_detection_publisher.publish(msg)
            self.get_logger().debug("Publishing: X: %.2f Y: %.2f Z: %.2f Yaw: %.2f Detected: False" % 
                                (msg.x_global, msg.y_global, msg.z_global, msg.yaw))
            
    def quat_get_yaw(self, q):
        q_w = q[0]
        q_x = q[1]
        q_y = q[2]
        q_z = q[3]
        return math.atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z))

def main():
    rclpy.init()
    line_tracker_node = LineTracker()
    try:
        rclpy.spin(line_tracker_node)
    except KeyboardInterrupt:
         print("Shutting Down") 
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
