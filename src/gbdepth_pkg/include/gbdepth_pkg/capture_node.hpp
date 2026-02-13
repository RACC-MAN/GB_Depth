#ifndef GBDEPTH_PKG__CAPTURE_NODE
#define GBDEPTH_PKG__CAPTURE_NODE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace std;

class CaptureNode : public rclcpp::Node
{
    public:
        explicit CaptureNode(const rclcpp::NodeOptions & options);
    
    private:
        void callback();
        
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        cv::VideoCapture cap;
        cv::Mat frame;

        string url_ = "http://172.17.134.88";
};

#endif