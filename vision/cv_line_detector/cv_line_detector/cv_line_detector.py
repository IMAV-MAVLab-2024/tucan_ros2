#! /usr/bin/env python3
#OPENCV
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tucan_msgs.msg import LineFollower # Define this message type if it doesn't exist yet

# Camera calibration parameters
camera_matrix = np.array([[528.673438813997, 0, 362.958493066534],
                          [0, 569.793218233108, 283.723935140803],
                          [0, 0, 1]])

dist_coeffs = np.array([0.138739907567143, -0.272661915942306, 0, 0, 0])

# Known line width in meters
line_width = 0.019  # 19mm

class LineTracker(Node):
    def __init__(self):
        super().__init__("cv_line_tracker")
        self.get_logger().info("CV Line Tracker Node has been started")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publisher
        self.line_detection_publisher = self.create_publisher(LineFollower, "/cv_line_detection", 1)
        self.bridge_for_CV = CvBridge()
        self.subscription = self.create_subscription(Image, "/down_camera_image", self.ImageLoop, 1)

    def ImageLoop(self, data):
        msg = LineFollower()
        img = self.bridge_for_CV.imgmsg_to_cv2(data)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define the range for the color blue
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find edges in the mask using Canny
        edges = cv2.Canny(mask, 50, 150, apertureSize=3)

        # Find lines in the image using Hough Line Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=10)

        if lines is not None:
            line_angles = []
            line_offsets = []
            height, width = img.shape[:2]
            middle_x = width / 2

            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                line_angles.append(angle)

                # Calculate the average position of the line
                line_center_x = (x1 + x2) / 2
                offset = line_center_x - middle_x
                line_offsets.append(offset)

                # Draw lines on the image for visualization
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if line_angles and line_offsets:
                avg_angle = np.mean(line_angles)
                avg_offset = np.mean(line_offsets)
                msg.angle = float(avg_angle)
                msg.avg_offset = float(avg_offset)

                self.line_detection_publisher.publish(msg)
                self.get_logger().debug(f"Publishing: Angle: {avg_angle:.2f} Offset: {avg_offset:.2f}")
        else:
            msg.angle = float('nan')
            msg.avg_offset = float('nan')
            self.line_detection_publisher.publish(msg)
            self.get_logger().debug("No line detected")

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
