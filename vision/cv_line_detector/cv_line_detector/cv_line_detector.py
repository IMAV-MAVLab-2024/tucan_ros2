#! /usr/bin/env python3
#OPENCV
import numpy as np
import cv2
import math
#ROS2
import rclpy
from rclpy.node import Node
from tucan_msgs.msg import LineFollower

Images=[]
N_SLICES = 4
x_direction = [0] * N_SLICES
y_direction = [0] * N_SLICES
yaw_angle = [0] * N_SLICES
# angle wrt y axis (forward is y axis)
fps = 15.
frame_width = 800
frame_height = 600

class LineDetector(Node):
    def __init__(self):
        super().__init__("cv_line_detector")
        self.get_logger().info("CV Line detection Node has been started")
        self.yaw_offset_publisher = self.create_publisher(LineFollower, "cv_line_detection",int(fps))
        self.ImageLoop()

    def RemoveBackground(self,image):
        up = 200
        # create NumPy arrays from the boundaries
        lower = np.array([0, 0, 0], dtype = "uint8")
        upper = np.array([up, up, up], dtype = "uint8")
            
        #----------------COLOR SELECTION-------------- (Remove any area that is whiter than 'upper')
        mask = cv2.inRange(image, lower, upper)
        image = cv2.bitwise_and(image, image, mask = mask)
        image = cv2.bitwise_not(image, image, mask = mask)
        image = (255-image)
        return image

    def SlicePart(self, im, images, slices):
        height, width = im.shape[:2]
        sl = int(height/slices)
        # print("Height: %d Width: %d" % (height,width))
        for i in range(slices):
            part = sl*i
            crop_img = im[part:part+sl, 0:width]
            # images[i].image = crop_img
            self.ProcessFrame(crop_img,i,part)

    def getContourCenter(self,contour):
            M = cv2.moments(contour)
            
            if M["m00"] == 0:
                return 0
            
            x = int(M["m10"]/M["m00"])
            y = int(M["m01"]/M["m00"])
            
            return [x,y]

    #This function returns the extent of tilt of the detected tape segment 
    def getContourExtent(self,contour):
            #We find the area of the detected tape
            area = cv2.contourArea(contour)
            #boundingRect is a straight rectangle, it doesn't consider the rotation of the tape
            x,y,w,h = cv2.boundingRect(contour)
            rect_area = w*h
            if rect_area > 0:
                return (float(area)/rect_area)

    def Aprox(self, a, b, error):
        if abs(a - b) < error:
            return True
        else:
            return False

    # Line angle from y axis given two points:
    def GetAngle(self, x1, y1, x2, y2): 
        if x2-x1 == 0:
            return 90
        else:
            return 90 - math.degrees(math.atan((y2-y1)/(x2-x1))) 

    def ProcessFrame(self,img,pos,part):	
        contourCenterX = 0
        MainContour = None
        
        #Convert to Gray Scale
        imgray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) 
        #Get Threshold
        ret, thresh = cv2.threshold(imgray,100,255,cv2.THRESH_BINARY_INV) 
        
        #Get contour
        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
        
        prev_MC = MainContour
        if contours:
            MainContour = max(contours, key=cv2.contourArea)
            
            height, width  = img.shape[:2]

            #Get X coordenate of the middle point
            middleX = int(width/2)
            #Get Y coordenate of the middle point 
            middleY = int(height/2) 
                
            prev_cX = contourCenterX
            if self.getContourCenter(MainContour) != 0:
                contourCenterX = self.getContourCenter(MainContour)[0]
                if abs(prev_cX-contourCenterX) > 5:
                    if abs(prev_cX-contourCenterX) > 5:
                        for i in range(len(contours)):
                            if self.getContourCenter(contours[i]) != 0:
                                tmp_cx = self.getContourCenter(contours[i])[0]
                                if self.Aprox(tmp_cx, prev_cX, 5) == True:
                                    MainContour = contours[i]
                                    if self.getContourCenter(MainContour) != 0:
                                        contourCenterX = self.getContourCenter(MainContour)[0]

            else:
                contourCenterX = 0
            raw_x_direction = int(middleX-contourCenterX)
            # weighted_x_direction =  int(raw_x_direction * self.getContourExtent(MainContour))
            raw_y_direction = int(part + middleY)
            x_direction[pos] = raw_x_direction
            y_direction[pos] = raw_y_direction

            #Draw Contour GREEN
            cv2.drawContours(img,MainContour,-1,(0,255,0),3)             
            #Draw dX circle WHITE
            cv2.circle(img, (contourCenterX, middleY), 7, (255,255,255), -1) 
            #Draw middle circle RED
            cv2.circle(img, (middleX, middleY), 3, (0,0,255), -1) 
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img,str(middleX-contourCenterX),(contourCenterX+20, middleY), font, 1,(200,0,200),2,cv2.LINE_AA)
            cv2.putText(img,"Weight:%.3f"%self.getContourExtent(MainContour),(contourCenterX+20, middleY+35), font, 0.5,(200,0,200),1,cv2.LINE_AA)						

    def ImageLoop(self):
        msg = LineFollower()

        # starting video streaming
        cv2.namedWindow('down_frame')

        # Create capture
        video_capture = cv2.VideoCapture(0)
        # Set camera properties
        # video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        # video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        # video_capture.set(cv2.CAP_PROP_FPS, fps)

        while True:
            img = video_capture.read()[1]
            img = self.RemoveBackground(img)
            final_offset = 0
            final_yaw_angle = 0
            self.SlicePart(img, Images, N_SLICES)
            for i in range(len(yaw_angle)):
                if i == N_SLICES-1:
                    yaw_angle[i] = self.GetAngle(x_direction[0], y_direction[0], x_direction[i], y_direction[i])
                else:
                    yaw_angle[i] = self.GetAngle(x_direction[i], y_direction[i], x_direction[i+1], y_direction[i+1])
            for i in range(len(x_direction)):
                final_offset += x_direction[i]
                final_yaw_angle += yaw_angle[i]
            #Process(img)						
            msg.avg_offset = final_offset/N_SLICES
            msg.angle = final_yaw_angle/N_SLICES
            self.yaw_offset_publisher.publish(msg)
            self.get_logger().info("Publishing: angle:%d offset:%d" % (msg.angle,msg.avg_offset))
            cv2.imshow("window_frame", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Clean up the connection
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    
    line_detector_node = LineDetector()
    try:
        rclpy.spin(line_detector_node)
    except KeyboardInterrupt:
         print("Shutting Down") 
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#pipeline = 'v4l2src device=/dev/video22 io-mode=4 ! video/x-raw, format=YUY2, width=640, height=480, pixel-aspect-ratio=1/1, framerate=15/1 ! videoconvert ! appsink'
#webcam2appsink_YUY2_640_480 = "v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=640, height=480, pixel-aspect-ratio=1/1, framerate=30/1 ! videoconvert ! appsink"
#video_capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
