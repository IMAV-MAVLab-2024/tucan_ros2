#! /usr/bin/env python3
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from tucan_msgs.msg import ObjectDetection  # 假设该消息类型已定义
from scipy.spatial.transform import Rotation as R

# 相机内参矩阵和畸变系数
camera_matrix = np.array([[528.673438813997, 0, 362.958493066534],
                          [0, 569.793218233108, 283.723935140803],
                          [0, 0, 1]])
dist_coeffs = np.array([0.138739907567143, -0.272661915942306, 0, 0, 0])

# 相机到机体坐标系的旋转和平移
R_frd_cam = np.array([[0, -1,  0], [1,  0,  0], [0,  0,  1]], dtype=np.float32)
t_frd_cam = np.array([-0.06, 0.0, 0.0], dtype=np.float32)

# 圆柱体的已知尺寸
cylinder_radius = 0.03  # 半径（米）
cylinder_length = 0.20  # 长度（米）

class CylinderDetector(Node):
    def __init__(self):
        super().__init__("cv_object_detector")
        self.get_logger().info("CV object detection Node has been started")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        self.bridge_for_CV = CvBridge()
        self.subscription = self.create_subscription(Image, "/down_camera_image", self.ImageLoop, 1)

        # 添加检测结果的发布器
        self.detection_publisher = self.create_publisher(ObjectDetection, "/cv_object_detection", 1)

        self.vehicle_odometry = None
        self.last_detection_timestamp = self.get_clock().now().to_msg()

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def ImageLoop(self, data):
        img = self.bridge_for_CV.imgmsg_to_cv2(data)
        
        # 将图像转换为HSV色彩空间
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 粉色的HSV范围
        lower_pink = np.array([140, 50, 50])
        upper_pink = np.array([170, 255, 255])

        # 创建粉色的掩码
        mask = cv2.inRange(hsv, lower_pink, upper_pink)

        # 找到轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 如果检测到了轮廓
        if len(contours) > 0 and self.vehicle_odometry is not None:
            # 找到最大的轮廓
            largest_contour = max(contours, key=cv2.contourArea)

            # 拟合椭圆
            if len(largest_contour) >= 5:
                ellipse = cv2.fitEllipse(largest_contour)
                (x, y), (major_axis, minor_axis), angle = ellipse

                # 估算圆柱体的位姿
                object_points = np.array([[-cylinder_length / 2, -cylinder_radius, 0],
                                          [cylinder_length / 2, -cylinder_radius, 0],
                                          [cylinder_length / 2, cylinder_radius, 0],
                                          [-cylinder_length / 2, cylinder_radius, 0]])

                image_points = np.array([[x, y],
                                         [x + major_axis / 2, y],
                                         [x, y + minor_axis / 2],
                                         [x - major_axis / 2, y]])

                # 使用solvePnP估计位姿
                success, rvec_cam, tvec_cam = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

                if success:
                    # 将相机坐标系下的位姿转换为机体坐标系
                    tvec_frd = np.dot(R_frd_cam, tvec_cam) + t_frd_cam
                    # tvec_frd 现在就是圆柱体在机体坐标系 (FRD) 下的位置
                    pos_x_frd, pos_y_frd, pos_z_frd = tvec_frd[0], tvec_frd[1], tvec_frd[2]
                    
                    # 使用四元数将机体坐标系下的位姿转换为NED坐标系
                    rotation_matrix = R.from_quat([self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3], self.vehicle_odometry.q[0]]).as_matrix()
                    tvec_ned = np.matmul(rotation_matrix, tvec_frd) + np.array([self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], self.vehicle_odometry.position[2]])

                    # 获取NED坐标系下的位置
                    pos_x, pos_y, pos_z = tvec_ned[0], tvec_ned[1], tvec_ned[2]

                    # 计算圆柱体在图像中的相对偏航角
                    img_center_x = img.shape[1] / 2
                    delta_x = x - img_center_x  # 图像中心到圆柱体中心的X轴距离
                    fov_x = np.degrees(2 * np.arctan2(img.shape[1], 2 * camera_matrix[0, 0]))  # 计算相机的水平视场角
                    yaw_offset = delta_x / img.shape[1] * fov_x  # 计算相对偏航角

                    # 将当前飞机的yaw加上相对偏航角得到新的yaw
                    current_yaw = R.from_quat([self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3], self.vehicle_odometry.q[0]]).as_euler('xyz')[2]
                    target_yaw = current_yaw + np.radians(yaw_offset)

                    # 创建消息对象并填充数据
                    detection_msg = ObjectDetection()
                    detection_msg.detected = True
                    detection_msg.x = pos_y_frd
                    detection_msg.y = pos_x_frd
                    detection_msg.x_global = pos_x
                    detection_msg.y_global = pos_y
                    detection_msg.z_global = pos_z
                    detection_msg.orientation.yaw = target_yaw  # 需要对准的偏航角

                    self.last_detection_timestamp = self.get_clock().now().to_msg()
                    detection_msg.last_detection_timestamp = self.last_detection_timestamp

                    # 发布检测结果
                    self.detection_publisher.publish(detection_msg)

                    self.get_logger().info(f"Cylinder detected at NED position: X: {pos_x}, Y: {pos_y}, Z: {pos_z}, Target Yaw: {np.degrees(target_yaw)}°")

        else:
            self.get_logger().info("No cylinder detected")

def main():
    rclpy.init()
    cylinder_detector_node = CylinderDetector()
    try:
        rclpy.spin(cylinder_detector_node)
    except KeyboardInterrupt:
        print("Shutting Down")
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
