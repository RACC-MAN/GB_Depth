#include "gbdepth_pkg/camera_node.hpp"

CameraNode::CameraNode(const rclcpp::NodeOptions & options) : Node("camera_node", options)
{
    int camera_num, publish_freq;
    declare_parameter("camera_num", 0);
    declare_parameter("publish_freq", 100);
    get_parameter("camera_num", camera_num);
    get_parameter("publish_freq", publish_freq);

    RCLCPP_INFO( get_logger(), "\nCamera Node Started.");
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera_img", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(publish_freq), std::bind(&CameraNode::callback, this));
    
    RCLCPP_INFO( get_logger(), "\nConecting to camera...");
    cap.open(camera_num);
    if(cap.isOpened())
    {
    cap.read(frame);
    int height = frame.rows;
    int width  = frame.cols;
    RCLCPP_INFO( get_logger(), "\nCamera {%d} is opened. height:%3d, width:%3d", camera_num, height, width);
    } 
}


void CameraNode::callback()
{
    sensor_msgs::msg::Image msg_out;
    std_msgs::msg::Header header_;
    cv_bridge::CvImage cv_img;


    cap.read(frame);
    // frame = cv::imread("path/to/image.file");
    if(frame.empty())
    {
        RCLCPP_INFO( get_logger(), "Frame is empty.");
        return;
    }

    header_.stamp = get_clock() -> now();
    cv_img = cv_bridge::CvImage(header_, sensor_msgs::image_encodings::BGR8, frame);
    cv_img.toImageMsg(msg_out);
    image_pub_->publish(msg_out);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode)