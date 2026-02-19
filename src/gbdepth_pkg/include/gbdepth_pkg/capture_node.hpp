#ifndef GBDEPTH_PKG__CAPTURE_NODE
#define GBDEPTH_PKG__CAPTURE_NODE

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>


class CaptureNode : public rclcpp::Node
{
public:
    explicit CaptureNode(const rclcpp::NodeOptions & options);

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void timerCallback();

    cv::VideoCapture cap_;
    cv::Mat frame_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int frame_id_;
    int counter_;
};

#endif
