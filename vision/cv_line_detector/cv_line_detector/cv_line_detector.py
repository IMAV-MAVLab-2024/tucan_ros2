#! /usr/bin/env python3
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import std_msgs.msg as std_msgs
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

        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value

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
        self.subscription_enable = self.create_subscription(std_msgs.Bool, "/cv_line_detector/enable", self.enable_callback, 1)

        #debug image publisher
        if self.debug:
            self.debug_image_publisher = self.create_publisher(Image, "/cv_line_detector/debug_image", 1)
            self.debug_image_publisher_blue = self.create_publisher(Image, "/cv_line_detector/debug_image_blue", 1)

        self.enabled = False

        self.last_lateral_offset = 0

        self.previous_x = 0
        self.previous_y = 0
        self.previous_yaw = 0
        self.previous_x_global = 0
        self.previous_y_global = 0

        self.vehicle_odometry = None

        self.last_detection_timestamp = self.get_clock().now().to_msg()

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def ImageLoop(self, data):
        if self.enabled:
            msg = LineFollower()
            img = self.bridge_for_CV.imgmsg_to_cv2(data, 'bgr8')

            # Convert image to HSV
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Define the color range for the blue color
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

            if self.debug:
                self.debug_image_publisher_blue.publish(self.bridge_for_CV.cv2_to_imgmsg(mask, 'mono8'))
            
            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Find the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                # Fit a line to the largest contour
                [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)

                #draw the line
                if self.debug:
                    cv2.line(img, (int(x - 1000 * vx), int(y - 1000 * vy)), (int(x + 1000 * vx), int(y + 1000 * vy)), (0, 255, 0), 2)
                    self.debug_image_publisher.publish(self.bridge_for_CV.cv2_to_imgmsg(img, 'bgr8'))


                # Calculate line's angle
                yaw = np.arctan2(-vx, vy) # Angle in radians

                if yaw > np.pi / 2:
                    yaw = yaw - np.pi
                elif yaw < -np.pi / 2:
                    yaw = yaw + np.pi
            
                # Calculate the center of the detected line in the image
                center_x = np.mean([largest_contour[:, 0, 0].min(), largest_contour[:, 0, 0].max()])
                center_y = np.mean([largest_contour[:, 0, 1].min(), largest_contour[:, 0, 1].max()])


                # Convert the line center to NED frame
                if self.vehicle_odometry is not None:

                    yaw_vehicle = self.quat_get_yaw(self.vehicle_odometry.q)

                    center_x = center_x - img.shape[1] / 2
                    # rotate the image_coord y offset to the ned frame
                    image_coord = np.array([0, center_x])
                    self.get_logger().debug("center_x: %s" % center_x)
                    rot_mat_2d = np.array([[np.cos(yaw_vehicle), -np.sin(yaw_vehicle)], [np.sin(yaw_vehicle), np.cos(yaw_vehicle)]])

                    lateral_offset_ned = np.dot(rot_mat_2d, image_coord)
                    lateral_offset_ned = lateral_offset_ned / np.linalg.norm(lateral_offset_ned)

                    self.last_detection_timestamp = self.get_clock().now().to_msg()
                    msg.last_detection_timestamp = self.last_detection_timestamp

                    msg.detected = True
                    msg.lateral_offset = float(center_x)
                    msg.x_offset_dir = float(lateral_offset_ned[0])
                    msg.y_offset_dir = float(lateral_offset_ned[1])
                    msg.x_picture = float(center_y)
                    msg.y_picture = float(center_x)
                    
                    if abs(yaw) > np.rad2deg(6):
                        msg.yaw = float(yaw + yaw_vehicle)
                    else:
                        msg.yaw = float(yaw_vehicle)

                    self.previous_yaw = msg.yaw
                    self.previous_x_global = msg.x_offset_dir
                    self.previous_y_global =  msg.y_offset_dir
                    self.previous_x = msg.x_picture
                    self.previous_y = msg.y_picture
                    self.last_lateral_offset = msg.lateral_offset

                    self.line_detection_publisher.publish(msg)
                    self.get_logger().info("Publishing: lateral_offset: %.2f x_offset_dir: %.2f y_offset_dir: %.2f yaw: %.2f Detected: True" %     
                                        (msg.lateral_offset, msg.x_offset_dir, msg.y_offset_dir, msg.yaw))
            else:
                msg.detected = False
                msg.x_offset_dir = float(self.previous_x_global)
                msg.y_offset_dir = float(self.previous_y_global)
                msg.lateral_offset = float(self.last_lateral_offset)
                msg.yaw = float(self.previous_yaw)  # Default yaw if no line detected
                msg.x_picture = float(self.previous_y)
                msg.y_picture = float(self.previous_x)
                msg.last_detection_timestamp = self.last_detection_timestamp
                self.line_detection_publisher.publish(msg)
                self.get_logger().info("Publishing: lateral_offset: %.2f x_offset_dir: %.2f y_offset_dir: %.2f yaw: %.2f Detected: False" %     
                                        (msg.lateral_offset, msg.x_offset_dir, msg.y_offset_dir, msg.yaw))

    def quat_get_yaw(self, q):
        q_w = q[0]
        q_x = q[1]
        q_y = q[2]
        q_z = q[3]
        return math.atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z))
    
    def enable_callback(self, msg):
        self.enabled = msg.data

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
