#include "gbdepth_pkg/capture_node.hpp"

CaptureNode::CaptureNode(const rclcpp::NodeOptions & options)
: Node("capture_node", options), frame_id_(0)
{
    int camera_num, publish_freq;

    declare_parameter("camera_num", 4);
    declare_parameter("publish_freq", 1000);

    get_parameter("camera_num", camera_num);
    get_parameter("publish_freq", publish_freq);

    RCLCPP_INFO(get_logger(), "Camera Node Started.");

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "camera_img", 10,
        std::bind(&CaptureNode::imageCallback, this, std::placeholders::_1));

    frame_id_ = 0;
    counter_ = 5;
    timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_freq),
        std::bind(&CaptureNode::timerCallback, this));
        
}

void CaptureNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    frame_ = cv_ptr->image;
}

void CaptureNode::timerCallback()
{
    if(counter_ > 0)
    {
        RCLCPP_INFO(get_logger(), "Capturing in %d seconds...", counter_);
        counter_--;
        return;
    }

    if(frame_.empty()) return;

    std::string filename = "capture_" +
        std::to_string(frame_id_) + ".png";

    cv::imwrite(filename, frame_);
    RCLCPP_INFO(get_logger(), "Saved %s", filename.c_str());
    frame_id_++;
    counter_ = 5;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CaptureNode)
