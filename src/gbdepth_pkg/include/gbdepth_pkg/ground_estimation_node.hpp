#ifndef GBDEPTH_PKG__GROUND_ESTIMATION_NODE
#define GBDEPTH_PKG__GROUND_ESTIMATION_NODE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "gbdepth_msgs/msg/depth_data.hpp"
#include "gbdepth_msgs/msg/ground_points.hpp"

#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class GroundEstimationNode : public rclcpp::Node
{
    public:
        explicit GroundEstimationNode(const rclcpp::NodeOptions & options);
    
    private:
        void depth_callback(const gbdepth_msgs::msg::DepthData::SharedPtr msg_in);
        void height_callback(const gbdepth_msgs::msg::DepthData::SharedPtr msg_in);

        rclcpp::Subscription<gbdepth_msgs::msg::DepthData>::SharedPtr depth_sub_;
        rclcpp::Subscription<gbdepth_msgs::msg::DepthData>::SharedPtr height_sub_;

        rclcpp::Publisher<gbdepth_msgs::msg::GroundPoints>::SharedPtr groundPoints_pub_;
        int EXE_STEP = 20; //step of executing ground-plane-area detection

        bool USE_MAP_DEBUG;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr normal_image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr floor_image_pub_;

        const float nx = 0.85, ny = -0.02, nz = 0.5;
        //x:0.85, y:-0.020, z:0.50

        gbdepth_msgs::msg::DepthData::SharedPtr height_data;
        float max_h, min_h, range_h;
        float TRESH_HEIGHT = 0.50;
        float TRESH_THETA = 0.8;
};

#endif