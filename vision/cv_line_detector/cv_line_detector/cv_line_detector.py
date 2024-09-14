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

        self.enabled = False

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
        if self.enabled:
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
                center_x = np.mean([largest_contour[:, 0, 0].min(), largest_contour[:, 0, 0].max()])
                center_y = np.mean([largest_contour[:, 0, 1].min(), largest_contour[:, 0, 1].max()])


                # Convert the line center to NED frame
                if self.vehicle_odometry is not None:
                    # Convert line center from FRD to NED
                    R_ned_frd = R.from_quat([self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3], self.vehicle_odometry.q[0]])
                    R_mat_ned_frd = R_ned_frd.as_matrix()
                    #R_mat_frd_ned = R_ned_frd.inv().as_matrix()
                    current_pos = np.array([self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], self.vehicle_odometry.position[2]])

                    # camera position in NED
                    pos_cam_frd = t_frd_cam
                    pos_cam_ned = np.matmul(R_mat_ned_frd, pos_cam_frd) + current_pos
                    
                    # point ray in NED
                    center_ray_cam = self.image_point_to_3d_ray([center_x, center_y], camera_matrix, dist_coeffs)
                    center_ray_frd = np.matmul(R_frd_cam, center_ray_cam) + t_frd_cam
                    center_ray_ned = np.matmul(R_mat_ned_frd, center_ray_frd) + pos_cam_ned

                    #print ("center_ray_ned: ", center_ray_ned)
                    self.get_logger().info("center_ray_cam: %.2f %.2f %.2f" % (center_ray_cam[0], center_ray_cam[1], center_ray_cam[2]))
                    self.get_logger().info("center_ray_frd: %.2f %.2f %.2f" % (center_ray_frd[0], center_ray_frd[1], center_ray_frd[2]))
                    self.get_logger().info("center_ray_ned: %.2f %.2f %.2f" % (center_ray_ned[0], center_ray_ned[1], center_ray_ned[2]))

                    # intersect the ray and the ground plane
                    Z_ground = 0
                    scale = (Z_ground - self.vehicle_odometry.position[2]) / center_ray_ned[2]
                    if scale > 0:
                        X_ground = center_ray_ned[0] * scale
                        Y_ground = center_ray_ned[1] * scale

                        self.get_logger().info("x and y realtive to current pos: %.2f %.2f" % (self.vehicle_odometry.position[0] - X_ground ,self.vehicle_odometry.position[1] - Y_ground))
                        
                        pos_x = X_ground
                        pos_y = Y_ground
                        pos_z = Z_ground

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

    def image_point_to_3d_ray(self, image_point, camera_matrix, dist_coeffs):
        # Convert the 2D image point to homogeneous coordinates (x, y, 1)
        homogeneous_image_point = np.array([image_point[0], image_point[1], 1.0], dtype=np.float32)
        
        # Compute the inverse of the camera matrix
        camera_matrix_inv = np.linalg.inv(camera_matrix)
        
        # Multiply the inverse of the camera matrix by the image point to get the ray direction
        ray_direction = camera_matrix_inv.dot(homogeneous_image_point)
        
        # Normalize the ray direction vector (optional, for direction purposes)
        ray_direction /= np.linalg.norm(ray_direction)
        
        return ray_direction

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
