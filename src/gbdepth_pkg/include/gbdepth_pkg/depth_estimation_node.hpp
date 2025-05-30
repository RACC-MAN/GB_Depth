#ifndef GBDEPTH_PKG__DEPTH_ESTIMATION_NODE
#define GBDEPTH_PKG__DEPTH_ESTIMATION_NODE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gbdepth_msgs/msg/depth_data.hpp"

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class DepthEstimationNode : public rclcpp::Node
{
    public:
        explicit DepthEstimationNode(const rclcpp::NodeOptions & options);

    private:
        void callback(const sensor_msgs::msg::Image::SharedPtr msg_in);
        std::vector<std::string> getOutputsNames( const cv::dnn::Net& net );

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::Publisher<gbdepth_msgs::msg::DepthData>::SharedPtr depth_pub_;
        rclcpp::Publisher<gbdepth_msgs::msg::DepthData>::SharedPtr depth_n_pub_;
        rclcpp::Publisher<gbdepth_msgs::msg::DepthData>::SharedPtr height_pub_;
        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

        cv::dnn::Net net ;

        float SENSOR_DIST;//[pix]
        float EYE_POSITION;//[m]
};

#endif