#! /usr/bin/env python3
#OPENCV
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# import imutils

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import  VehicleOdometry, VehicleLocalPosition, VehicleStatus

from scipy.spatial.transform import Rotation as R

#ROS2
import rclpy
from rclpy.node import Node
from tucan_msgs.msg import ARMarker, Mode

fx = 1.0  # 焦距
fy = 1.0  # 焦距
cx = 0.5  # 光学中心
cy = 0.5  # 光学中心

# 摄像头内参矩阵（假设已经通过标定获得）
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]])
dist_coeffs = np.zeros((4, 1))  # 假设没有畸变

# ArUco 标记的真实尺寸（15cm）
marker_size = 0.15  # 单位: 米


class MarkerDetector(Node):
    def __init__(self):
        super().__init__("cv_aruco_detector")
        self.get_logger().info("CV AR detection Node has been started")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.vehicle_odometry = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.flight_mode_subscriber = self.create_subscription(Mode, '/mission_state', self.flight_mode_callback,1)


        self.yaw_offset_publisher = self.create_publisher(ARMarker, "/cv_aruco_detection", 1)
        self.bridge_for_CV = CvBridge()
        self.subscription = self.create_subscription(Image, "/down_camera_image", self.ImageLoop, 1)

        # Previoud X and Y
        self.previous_x = 0
        self.previous_y = 0

        self.previous_x_global = 0
        self.previous_y_global = 0
        self.previous_z_global = 0

        self.previous_id = 0

        self.vehicle_local_position = None
        self.vehicle_odometry = None
        # self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        # the aruco recodring positions
        self.aruco_positions = {}

    # def flight_mode_callback(self, msg):
    #     if msg.mode == 11: # tasks are done
    #         self.get_logger().info("-----------------Tasks are done-----------------")
    #         with open('aruco_positions.json', 'r') as f:
    #             self.aruco_positions = json.load(f)
    
    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle_odometry topic subscriber."""
        self.vehicle_odometry = msg

    def ImageLoop(self,data):
        msg = ARMarker()
        img = self.bridge_for_CV.imgmsg_to_cv2(data)
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        arucoParams = cv2.aruco.DetectorParameters()
        arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = arucoDetector.detectMarkers(img)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):

                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_size, camera_matrix, dist_coeffs)
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                if len(rvec)>0:

                    rvec = rvec[0][0]
                    tvec = tvec[0][0]
                    
                    # 转换到 Body Frame
                    tvec_body = np.dot(R.from_quat(self.vehicle_odometry.q).as_matrix(), tvec)

                    # 转换到 Inertial Frame
                    pos_x = self.vehicle_odometry.position[0] + tvec_body[0]
                    pos_y = self.vehicle_odometry.position[1] + tvec_body[1]
                    pos_z = self.vehicle_odometry.position[2] + tvec_body[2]
                    self.previous_x_global = pos_x
                    self.previous_y_global = pos_y
                    self.previous_z_global = pos_z
                    
                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                height, width  = img.shape[:2]

                #Get X coordenate of the middle point
                middleX = int(width/2)
                #Get Y coordenate of the middle point 
                middleY = int(height/2) 

                msg.id = int(markerID)
                msg.detected = True
                msg.x = int(cX-middleX)
                msg.y = int(cY-middleY)  
                self.previous_x = msg.x
                self.previous_y = msg.y
                msg.x_global = pos_x
                msg.y_global = pos_y
                msg.z_global = pos_z

                self.aruco_positions[markerID] = (pos_x, pos_y, pos_z)

                self.previous_id = msg.id
                                  
        else:
            msg.id = self.previous_id
            msg.detected = False
            msg.x = self.previous_x
            msg.y = self.previous_y
            msg.x_global = self.previous_x_global
            msg.y_global = self.previous_y_global
            msg.z_global = self.previous_z_global

        self.yaw_offset_publisher.publish(msg)
        # self.get_logger().debug("Publishing: Marker ID: %d X: %d Y: %d" % (msg.id, msg.x, msg.y))
        self.get_logger().debug("Publishing: Marker ID: %d X: %d Y: %d Detected: %s" % 
                                (msg.id, msg.x, msg.y, msg.detected))

        # cv2.imshow("window_frame", img)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     return

def main():
    rclpy.init()
    
    marker_detector_node = MarkerDetector()
    try:
        rclpy.spin(marker_detector_node)
    except KeyboardInterrupt:
         print("Shutting Down") 
    rclpy.shutdown()
    # Clean up the connection
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

#pipeline = 'v4l2src device=/dev/video22 io-mode=4 ! video/x-raw, format=YUY2, width=640, height=480, pixel-aspect-ratio=1/1, framerate=15/1 ! videoconvert ! appsink'
#webcam2appsink_YUY2_640_480 = "v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640, height=480, pixel-aspect-ratio=1/1, framerate=30/1 ! videoconvert ! appsink"
#video_capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
