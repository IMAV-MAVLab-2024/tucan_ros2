#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class DriverCamera : public rclcpp::Node
{
public:
    DriverCamera() : Node("camera_driver_node")
    {
        // Declare parameters
        this->declare_parameter<int>("camera_id", 22);  // down = 22, front = 31, laptop = 0
        this->declare_parameter<bool>("compress", false);
        this->declare_parameter<int>("FPS", 15);
        this->declare_parameter<int>("frame_width", 800);
        this->declare_parameter<int>("frame_height", 600);
        this->declare_parameter<std::string>("topic_name", "/camera_image");

        // Get parameters
        camera_id_ = this->get_parameter("camera_id").as_int();
        compress_ = this->get_parameter("compress").as_bool();
        fps_ = this->get_parameter("FPS").as_int();
        frame_width_ = this->get_parameter("frame_width").as_int();
        frame_height_ = this->get_parameter("frame_height").as_int();
        topic_name_ = this->get_parameter("topic_name").as_string();

        // Log the parameters
        RCLCPP_INFO(this->get_logger(), "Starting camera driver node with the following parameters:");
        RCLCPP_INFO(this->get_logger(), "Camera ID: %d", camera_id_);
        RCLCPP_INFO(this->get_logger(), "Compress: %s", compress_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "FPS: %d", fps_);
        RCLCPP_INFO(this->get_logger(), "Frame Width: %d", frame_width_);
        RCLCPP_INFO(this->get_logger(), "Frame Height: %d", frame_height_);
        RCLCPP_INFO(this->get_logger(), "Topic Name: %s", topic_name_.c_str());

        // Open the camera
        cap_.open(camera_id_);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            return;
        }

        // Set camera properties
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        // Initialize image transport
        image_transport::ImageTransport it(this->shared_from_this());

        // Publisher for raw images
        raw_image_publisher_ = it.advertise(topic_name_ + "/raw", 1);

        // Publisher for Theora compressed images
        if (compress_) {
            theora_image_publisher_ = it.advertise(topic_name_ + "/theora", 1);
        }

        // Publisher for JPEG compressed images (using CompressedImage)
        compressed_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name_ + "/compressed", 1);

        // Create a timer to capture and publish frames at the desired rate
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / fps_),
                                         std::bind(&DriverCamera::get_camera_images, this));
    }

private:
    void get_camera_images()
    {
        // Capture frame from camera
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
            return;
        }

        // Prepare the header for both messages
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();

        // Publish raw image
        sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        raw_image_publisher_.publish(image_msg);

        // Publish Theora-compressed image if enabled
        if (compress_) {
            theora_image_publisher_.publish(image_msg);
        }

        // Publish JPEG-compressed image (as sensor_msgs::msg::CompressedImage)
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf);
        auto compressed_image_msg = sensor_msgs::msg::CompressedImage();
        compressed_image_msg.header = header;
        compressed_image_msg.format = "jpeg";
        compressed_image_msg.data = std::move(buf);
        compressed_image_publisher_->publish(compressed_image_msg);
    }

    // Parameters
    int camera_id_;
    bool compress_;
    int fps_;
    int frame_width_;
    int frame_height_;
    std::string topic_name_;

    // OpenCV video capture object
    cv::VideoCapture cap_;

    // Image transport publishers
    image_transport::Publisher raw_image_publisher_;
    image_transport::Publisher theora_image_publisher_;

    // Publisher for compressed image (sensor_msgs::msg::CompressedImage)
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_publisher_;

    // Timer for frame publishing
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriverCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
