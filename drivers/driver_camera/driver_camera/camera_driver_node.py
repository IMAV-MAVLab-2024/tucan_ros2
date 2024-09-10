import rclpy
from rclpy.node import Node
import cv2 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

class DriverCamera(Node):
    """Camera driver node"""
    def __init__(self):
        super().__init__('camera_driver_node')
        # Default for laptop testing
        self.declare_parameter('camera_id', 22)  # down = 22, front = 31, laptop = 0
        self.declare_parameter('compress', False)
        self.declare_parameter('FPS', 15)
        self.declare_parameter('frame_width', 800)
        self.declare_parameter('frame_height', 600)
        self.declare_parameter('topic_name', "/camera_image")

        self.compress = self.get_parameter('compress').get_parameter_value().bool_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.FPS = self.get_parameter('FPS').get_parameter_value().integer_value
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.get_logger().info('Starting camera driver node with the following parameters:')
        self.get_logger().info('Camera ID: ' + str(self.camera_id))
        self.get_logger().info('Compress: ' + str(self.compress))
        self.get_logger().info('FPS: ' + str(self.FPS))
        self.get_logger().info('Frame Width: ' + str(self.frame_width))
        self.get_logger().info('Frame Height: ' + str(self.frame_height))
        self.get_logger().info('Topic Name: ' + str(self.topic_name))

        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            return

        if not self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width):
            self.get_logger().error('Failed to set frame width')
        if not self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height):
            self.get_logger().error('Failed to set frame height')
        if not self.cap.set(cv2.CAP_PROP_FPS, self.FPS):
            self.get_logger().error('Failed to set FPS')
        #if not self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1):
        #    self.get_logger().error('Failed to set buffer size')

        if self.compress:
            self.image_publisher = self.create_publisher(CompressedImage, self.topic_name, 1)
        else:
            self.image_publisher = self.create_publisher(Image, self.topic_name, 1)

        # only keep most recent image, we dont care about old images 
        self.bridge_for_CV = CvBridge()

        while rclpy.ok():
            ret, frame = self.cap.read()     
            if ret == True:
                #self.get_logger().info('Captured frame successfully')

                if self.compress:
                    #self.get_logger().info('Compressing image')
                    # Compress image using JPEG format
                    success, encoded_image = cv2.imencode('.jpg', frame)
                    if success:
                        compressed_image_msg = CompressedImage()
                        compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
                        compressed_image_msg.format = "jpeg"
                        compressed_image_msg.data = encoded_image.tobytes()  # Convert to bytes
                        self.image_publisher.publish(compressed_image_msg)
                    else:
                        self.get_logger().error('Failed to compress image')
                else:
                    #self.get_logger().debug('Publishing uncompressed image')
                    self.image_publisher.publish(self.bridge_for_CV.cv2_to_imgmsg(frame, encoding="rgb8"))
                rclpy.spin_once(self)

        self.cap.release()
            
def main():
    rclpy.init()
    camera_driver_node = DriverCamera()

    try:
        rclpy.spin(camera_driver_node)
    except KeyboardInterrupt:
         print("Shutting Down") 
    rclpy.shutdown()

    
    camera_driver_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()