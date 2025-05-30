#ifndef GBDEPTH_PKG__CAMERA_NODE
#define GBDEPTH_PKG__CAMERA_NODE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;

class CameraNode : public rclcpp::Node
{
    public:
        explicit CameraNode(const rclcpp::NodeOptions & options);
    
    private:
        void callback();
        
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        cv::VideoCapture cap;
        cv::Mat frame;
};

#endif