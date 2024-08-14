#! /usr/bin/env python3
#OPENCV
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# import imutils
#ROS2
import rclpy
from rclpy.node import Node
from tucan_msgs.msg import ARMarker

Images=[]
fps = 15.

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
}

class MarkerDetector(Node):
    def __init__(self):
        super().__init__("cv_aruco_detector")
        self.get_logger().info("CV AR detection Node has been started")
        self.yaw_offset_publisher = self.create_publisher(ARMarker, "cv_aruco_detector", int(fps))
        self.bridge_for_CV = CvBridge()
        self.subscription = self.create_subscription(Image, "laptop_camera_image", self.ImageLoop, int(fps))

    def RemoveBackground(self,image):
        up = 100
        # create NumPy arrays from the boundaries
        lower = np.array([0, 0, 0], dtype = "uint8")
        upper = np.array([up, up, up], dtype = "uint8")
            
        #----------------COLOR SELECTION-------------- (Remove any area that is whiter than 'upper')
        mask = cv2.inRange(image, lower, upper)
        image = cv2.bitwise_and(image, image, mask = mask)
        image = cv2.bitwise_not(image, image, mask = mask)
        image = (255-image)
        return image

    def ImageLoop(self,data):
        msg = ARMarker()
        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 600 pixels
        # img = imutils.resize(img, width=1000)
        img = self.bridge_for_CV.imgmsg_to_cv2(data)
        arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_5X5_1000"])
        arucoParams = cv2.aruco.DetectorParameters_create()

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection
                cv2.line(img, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(img, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(img, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(img, bottomLeft, topLeft, (0, 255, 0), 2)

                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(img, (cX, cY), 4, (0, 0, 255), -1)

                height, width  = img.shape[:2]

                #Get X coordenate of the middle point
                middleX = int(width/2)
                #Get Y coordenate of the middle point 
                middleY = int(height/2) 

                #Draw middle circle RED
                cv2.circle(img, (middleX, middleY), 3, (0,0,255), -1) 

                # draw the ArUco marker ID on the frame
                cv2.putText(img, str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

                msg.id = int(markerID)
                msg.x = int(cX-middleX)
                msg.y = int(cY-middleY)                    
        else:
            msg.id = 0
            msg.x = 0
            msg.y = 0
        self.yaw_offset_publisher.publish(msg)
        self.get_logger().info("Publishing: Marker ID: %d X: %d Y: %d" % (msg.id, msg.x, msg.y))

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
