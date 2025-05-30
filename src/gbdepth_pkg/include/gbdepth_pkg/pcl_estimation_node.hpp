#ifndef GBDEPTH_PKG__PCL_ESTIMATION_NODE
#define GBDEPTH_PKG__PCL_ESTIMATION_NODE

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "gbdepth_msgs/msg/depth_data.hpp"
#include "gbdepth_msgs/msg/ground_points.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

using namespace std;

class PclEstimationNode : public rclcpp::Node
{
    public:
        explicit PclEstimationNode(const rclcpp::NodeOptions & options);

    private:
        void depth_callback(const gbdepth_msgs::msg::DepthData::SharedPtr msg_in);
        void image_callback(sensor_msgs::msg::Image::SharedPtr msg_in);
        void groundPoints_callback(const gbdepth_msgs::msg::GroundPoints::SharedPtr msg_in);
        void least_squares_calc(vector<vector<float>> data, int N);

        rclcpp::Subscription<gbdepth_msgs::msg::DepthData>::SharedPtr depth_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImage_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorImage_sub_;
        rclcpp::Subscription<gbdepth_msgs::msg::GroundPoints>::SharedPtr groundArea_sub_;
        bool colorImage_avarable, groundArea_avarable;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclRaw_pub_;

        gbdepth_msgs::msg::DepthData::SharedPtr depth_data;

        cv::Mat camera_img, depth_img;
        int image_step, image_elemSize;

        float pointX, pointY, pointZ;
        float posX, posY, posZ;

        vector<int> sample_point_x, sample_point_y;
        int point_num ;
        gbdepth_msgs::msg::GroundPoints::SharedPtr sample_points;
        float SCALE_VALUE = 0.001;
        float SHIFT_VALUE = 0.1;

        float SENSOR_DIST ;//[pix]
        float EYE_POSITION ;//[m]
        string FRAME_ID ;
        int PIXEL_STEP ;
};

#endif