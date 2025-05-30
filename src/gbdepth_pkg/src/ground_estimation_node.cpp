#include "gbdepth_pkg/ground_estimation_node.hpp"

GroundEstimationNode::GroundEstimationNode(const rclcpp::NodeOptions & options) : Node("ground_estimation_node", options)
{
    RCLCPP_INFO( get_logger(), "\nGround Estimation Node Started.");

    height_sub_ = create_subscription<gbdepth_msgs::msg::DepthData>("/height_data", 10,
                            bind(&GroundEstimationNode::height_callback, this, placeholders::_1));
    groundPoints_pub_ = create_publisher<gbdepth_msgs::msg::GroundPoints>("ground_area", 10);

    RCLCPP_INFO( get_logger(), "\nMap Debug is avarable");
    depth_sub_ = create_subscription<gbdepth_msgs::msg::DepthData>("/depth_n_data", 10,
                            bind(&GroundEstimationNode::depth_callback, this, placeholders::_1));
    normal_image_pub_ = create_publisher<sensor_msgs::msg::Image>("normalMap", 10);
    floor_image_pub_ = create_publisher<sensor_msgs::msg::Image>("floorMap", 10);
}

void GroundEstimationNode::depth_callback(const gbdepth_msgs::msg::DepthData::SharedPtr msg_in)
{
    if (!height_data) {
        RCLCPP_INFO(this->get_logger(), "Height data is not received yet!");
        return;
    }

    gbdepth_msgs::msg::GroundPoints ground_area;
    ground_area.step = EXE_STEP;
    int size = 0;
    
    int width = msg_in->width;
    int height = msg_in->height;
    cv::Mat normalMap(height, width, CV_8UC3);
    int step = (int)normalMap.step;
    int elemSize = (int)normalMap.elemSize();

    cv::Mat floorMap(height, width, CV_8UC3);

    int accesspoint;

    for(int i=1; i<height-1; i+=1)
    {
        for( int j=1; j<width-1; j+=1)
        {
            float Dx1 = msg_in->data[(i+1)*(width) + j];
            float Dx2 = msg_in->data[(i-1)*(width) + j];
            float Dy1 = msg_in->data[i*width + (j-1)];
            float Dy2 = msg_in->data[i*width + (j+1)];

            // float x =  2*(Dx2-Dx1)*100000;
            // float y = -2*(Dy2-Dy1)*100000;
            float x =  2*(Dx2-Dx1)*1000;
            float y = -2*(Dy2-Dy1)*1000;
            float z = 4 ;
            float norm_inv = 1/sqrt(x*x+y*y+16);

            x *= norm_inv;
            y *= norm_inv;
            z *= norm_inv;

            int b = (int)( 127*x + 127); //axis x -> blue
            int g = (int)( 127*y + 127); //axis y -> green
            int r = (int)( 127*z + 127); //axis z -> red

            accesspoint = i*step + j*elemSize;
            
            normalMap.data[ accesspoint + 0] = b;
            normalMap.data[ accesspoint + 1] = g;
            normalMap.data[ accesspoint + 2] = r;

            // float height_level = height_data->data[i*width + j] - min_h;
            // height_level = height_level / range_h;
            float height_level = height_data->data[i*width + j];
            height_level = height_level / max_h;
            if(height_level < TRESH_HEIGHT ) height_level = 0.0;

            float cos_theta = x*nx + y*ny + z*nz;
            if( cos_theta < TRESH_THETA) cos_theta = 0; 

            float ground_level = height_level * cos_theta;

            floorMap.data[ accesspoint + 0] = ground_level * b;
            floorMap.data[ accesspoint + 1] = ground_level * g;
            floorMap.data[ accesspoint + 2] = ground_level * r;

            // floorMap.data[ accesspoint + 0] = height_level * 255;
            // floorMap.data[ accesspoint + 1] = height_level * 255;
            // floorMap.data[ accesspoint + 2] = height_level * 255;

            if(j%EXE_STEP==0 && i%EXE_STEP==0 )
            {
                if((true * ground_level)!=0)
                {
                    // RCLCPP_WARN(this->get_logger(), "%3d, %3d", i, j);
                    // floorMap.data[ accesspoint + 0] = 255;
                    // floorMap.data[ accesspoint + 1] = 255;
                    // floorMap.data[ accesspoint + 2] = 255;
                    ground_area.x.push_back(j);
                    ground_area.y.push_back(i);
                    size++;
                } 
            }
        }
    }

    sensor_msgs::msg::Image::SharedPtr msg_out;

    normalMap.convertTo( normalMap, CV_8UC3);
    msg_out  = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", normalMap).toImageMsg();
    normal_image_pub_->publish(*msg_out.get());
    
    floorMap.convertTo( floorMap, CV_8UC3);
    msg_out  = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", floorMap).toImageMsg();
    floor_image_pub_->publish(*msg_out.get());

    ground_area.size = size;
    groundPoints_pub_->publish(ground_area);
}

void GroundEstimationNode::height_callback(const gbdepth_msgs::msg::DepthData::SharedPtr msg_in)
{
    height_data = msg_in;
    max_h = msg_in->max;
    min_h = msg_in->min;
    range_h = max_h - min_h;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GroundEstimationNode)