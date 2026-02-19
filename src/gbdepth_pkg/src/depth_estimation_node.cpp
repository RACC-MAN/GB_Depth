#include "gbdepth_pkg/depth_estimation_node.hpp"

DepthEstimationNode::DepthEstimationNode(const rclcpp::NodeOptions &options) : Node("depth_estimation_node", options)
{
    // Ros Setup
    RCLCPP_INFO( this->get_logger(), "\nDepth Estimation Node Started.");
    
    this->declare_parameter("sensor_dist" ,0.0);
    this->declare_parameter("eye_position", 0.0);
    this->get_parameter("sensor_dist" , SENSOR_DIST );
    this->get_parameter("eye_position", EYE_POSITION);
    RCLCPP_INFO(this->get_logger(), "\nsensor_dist:%f, eye_position:%f", SENSOR_DIST, EYE_POSITION);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera_img", 10,
                            bind(&DepthEstimationNode::callback, this, placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_img", 10);
    depth_n_pub_ = this->create_publisher<gbdepth_msgs::msg::DepthData>("depth_n_data", 10);
    depth_pub_ = this->create_publisher<gbdepth_msgs::msg::DepthData>("depth_data", 10);
    height_pub_ = this->create_publisher<gbdepth_msgs::msg::DepthData>("height_data", 10);
    
    // Read Network
    // const std::string model = "./src//model-f6b98070.onnx"; // MiDaS v2.1 Large
    const std::string model = "/home/keisoku/gbdepth_ws/models/midas/model-small.onnx"; // MiDaS v2.1 Small
    net = cv::dnn::readNet( model );
    if( net.empty() ){
        RCLCPP_INFO(this->get_logger(), "net is empty");
    }

    // Set Preferable Backend and Target
    // net.setPreferableBackend( cv::dnn::DNN_BACKEND_OPENCV );
    // net.setPreferableTarget( cv::dnn::DNN_TARGET_CPU );
    net.setPreferableBackend( cv::dnn::DNN_BACKEND_CUDA );
    net.setPreferableTarget( cv::dnn::DNN_TARGET_CUDA );
}

void DepthEstimationNode::callback(const sensor_msgs::msg::Image::SharedPtr msg_in)
{
    // Read Frame
    cv::Mat input;
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(msg_in, msg_in->encoding);
    input = cv_img->image;
    // cv::cvtColor(input, input, cv::COLOR_GRAY2BGR);
    if( input.channels() == 4 ){
        cv::cvtColor( input, input, cv::COLOR_BGRA2BGR );
    }

    // Create Blob from Input Image
    // MiDaS v2.1 Large ( Scale : 1 / 255, Size : 384 x 384, Mean Subtraction : ( 123.675, 116.28, 103.53 ), Channels Order : RGB )
    // cv::Mat blob = cv::dnn::blobFromImage( input, 1 / 255.f, cv::Size( 384, 384 ), cv::Scalar( 123.675, 116.28, 103.53 ), true, false );
    // // MiDaS v2.1 Small ( Scale : 1 / 255, Size : 256 x 256, Mean Subtraction : ( 123.675, 116.28, 103.53 ), Channels Order : RGB )
    cv::Mat blob = cv::dnn::blobFromImage( input, 1 / 255.f, cv::Size( 256, 256 ), cv::Scalar( 123.675, 116.28, 103.53 ), true, false );

    // Set Input Blob
    net.setInput( blob );

    // Run Forward Network
    cv::Mat output = net.forward( getOutputsNames( net )[0] );

    // Convert Size to 384x384 from 1x384x384
    const std::vector<int32_t> size = { output.size[1], output.size[2] };
    output = cv::Mat( static_cast<int32_t>( size.size() ), &size[0], CV_32F, output.ptr<float>() );

    // Resize Output Image to Input Image Size
    cv::resize( output, output, input.size() );
    int height = input.rows;
    int width = input.cols;
    RCLCPP_INFO(this->get_logger(), "\nheight:%d, width:%d", height, width);
    float ch = height/2.0;
    float cw = width/2.0;
    // RCLCPP_INFO(this->get_logger(), "h:%3d, w:%3d", height, width);
    
    double minDe, maxDe; 
    cv::minMaxLoc( output, &minDe, &maxDe);
    double range = maxDe - minDe;
    
    gbdepth_msgs::msg::DepthData depth_n_data;
    depth_n_data.width = width;
    depth_n_data.height = height;
    depth_n_data.max = maxDe;
    depth_n_data.min = minDe;

    gbdepth_msgs::msg::DepthData depth_data;
    depth_data.width = width;
    depth_data.height = height;
    depth_data.max = maxDe;
    depth_data.min = minDe;

    gbdepth_msgs::msg::DepthData height_data;
    height_data.width = width;
    height_data.height = height;
    float max_h=0, min_h=0;
    // RCLCPP_INFO(this->get_logger(), "h:%d, w:%d", height, width);

    for(int i=0; i<height; i+=1)
    {
        for( int j=0; j<width; j+=1)
        {
            float De = output.at<float>(cv::Point(j, i));
            float value = (maxDe - De)/range;
            depth_n_data.data.push_back(value);
            // value = 1000/De;
            value = 1/De;
            depth_data.data.push_back(value);

            float pix_y = i-ch, pix_x = j-cw;
            // float pix_dist_inv = 1/sqrt(SENSOR_DIST*SENSOR_DIST + pix_x*pix_x + pix_y*pix_y);
            float pix_dist_inv = 1/SENSOR_DIST;
            // value = 1/De;
            value = value*pix_y*pix_dist_inv;
            if(value > max_h) max_h = value;
            if(min_h > value) min_h = value;
            height_data.data.push_back(value);
        }
    }
    height_data.max = max_h;
    height_data.min = min_h;

    // Visualize Output Image
    // 1. Normalize ( 0.0 - 1.0 )
    // 2. Scaling ( 0 - 255 )
    output.convertTo( output, CV_32F, 1.0 / range, - ( minDe / range ) );
    output.convertTo( output, CV_8U, 255.0 );

    sensor_msgs::msg::Image depth_out;
    cv_img->image = output;
    cv_img->encoding = "mono8";
    cv_img->toImageMsg(depth_out);
    image_pub_->publish(depth_out);

    depth_n_pub_->publish(depth_n_data);
    depth_pub_->publish(depth_data);
    height_pub_->publish(height_data);
}

std::vector<std::string> DepthEstimationNode::getOutputsNames(const cv::dnn::Net &net)
{
    static std::vector<std::string> names;
    if( names.empty() ){
        std::vector<int32_t> out_layers = net.getUnconnectedOutLayers();
        std::vector<std::string> layers_names = net.getLayerNames();
        names.resize( out_layers.size() );
        for( size_t i = 0; i < out_layers.size(); ++i ){
            names[i] = layers_names[out_layers[i] - 1];
        }
    }
    return names;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DepthEstimationNode)