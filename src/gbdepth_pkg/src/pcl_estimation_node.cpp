#include "gbdepth_pkg/pcl_estimation_node.hpp"

PclEstimationNode::PclEstimationNode(const rclcpp::NodeOptions & options) : Node("pcl_estimation_node", options)
{
            RCLCPP_INFO( this->get_logger(), "\nPCL estimation Node Started");

            this->declare_parameter("sensor_dist" ,697.0);
            this->declare_parameter("eye_position", 0.735);
            this->declare_parameter("frame_id", "map");
            this->declare_parameter("pixel_step", 4);
            this->declare_parameter("pos_x", 0.0);
            this->declare_parameter("pos_y", 0.0);
            this->declare_parameter("pos_z", 0.0);

            this->get_parameter("sensor_dist" , SENSOR_DIST );
            this->get_parameter("eye_position", EYE_POSITION);
            this->get_parameter("frame_id", FRAME_ID);
            this->get_parameter("pixel_step", PIXEL_STEP);
            this->get_parameter("pos_x", posX);
            this->get_parameter("pos_y", posY);
            this->get_parameter("pos_z", posZ);

            RCLCPP_INFO(this->get_logger(), "\nsensor_dist:%f, eye_position:%f, frame:%s", SENSOR_DIST, EYE_POSITION, FRAME_ID.c_str());
            RCLCPP_INFO(this->get_logger(), "\npos_x:%f, pos_y:%f, pos_z:%f", posX, posY, posZ);

            depth_sub_ = this->create_subscription<gbdepth_msgs::msg::DepthData>("/depth_data", 10,
                                        bind(&PclEstimationNode::depth_callback, this, placeholders::_1));

            colorImage_avarable = false;
            colorImage_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera_img", 10,
                                        bind(&PclEstimationNode::image_callback, this, placeholders::_1));

            groundArea_avarable = false;
            groundArea_sub_ = this->create_subscription<gbdepth_msgs::msg::GroundPoints>("/ground_area", 10,
                                        bind(&PclEstimationNode::groundPoints_callback, this, placeholders::_1));

            pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_data", 10);
            pclRaw_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pclRaw_data", 10);
}

void PclEstimationNode::depth_callback(gbdepth_msgs::msg::DepthData::SharedPtr msg_in)
{
    if(!colorImage_avarable || !groundArea_avarable) 
    {
        RCLCPP_INFO(this->get_logger(), "not ready");
        return;
    }

    int width = msg_in->width;
    int height = msg_in->height;
    float ch = height/2;
    float cw = width/2;


    vector<vector<float>> sample_data{};
    int point_num = sample_points->size;

    for(int i=0; i<point_num; i++)
    {
        vector<float> tmp_vec={0,0};
        int h = sample_points->y[i];
        int w = sample_points->x[i];
        // RCLCPP_INFO(this->get_logger(), "h:%3d, w:%3d", h, w);

        tmp_vec[0] = msg_in->data[h*width + w];
        
        h-=ch;
        w-=cw;
        float tmp_w = w/h;
        float tmp_SD = SENSOR_DIST/h;
        tmp_vec[1] = EYE_POSITION * sqrt(1 + tmp_w*tmp_w + tmp_SD*tmp_SD);
        sample_data.push_back(tmp_vec);
    }
    least_squares_calc(sample_data, point_num);
    // RCLCPP_INFO(this->get_logger(), "scale:%f, shift:%f", SCALE_VALUE, SHIFT_VALUE);

    // SCALE_VALUE = 1;
    // SHIFT_VALUE = 0;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_data;
    for(int i=0; i<height; i+=PIXEL_STEP)
    {
        for(int j=0; j<width; j+=PIXEL_STEP)
        {
            pcl::PointXYZRGB new_point;

            float De_raw = msg_in->data[i*width + j];

            float De = SCALE_VALUE*De_raw + SHIFT_VALUE;

            float pix_y = i-ch, pix_x = j-cw;
            float pix_dist_inv = 1/SENSOR_DIST;
            pointX = posX + De*SENSOR_DIST*pix_dist_inv;
            pointY = posY + -De*pix_x*pix_dist_inv;     
            pointZ = posZ + -De*pix_y*pix_dist_inv;     
            
            new_point.x = pointX;
            new_point.y = pointY;
            new_point.z = pointZ;

            int accesspoint = i*image_step + j*image_elemSize;
            new_point.r = camera_img.data[ accesspoint + 0];
            new_point.g = camera_img.data[ accesspoint + 1];
            new_point.b = camera_img.data[ accesspoint + 2];

            pcl_data.points.push_back(new_point);
        }
    }

    auto pcl_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pcl_data, *pcl_msg);
    pcl_msg->header.frame_id = FRAME_ID;
    pcl_msg->header.stamp = now();
    pcl_pub_->publish(*pcl_msg);
}

void PclEstimationNode::image_callback(sensor_msgs::msg::Image::SharedPtr msg_in)
{
    colorImage_avarable = true;
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(msg_in, msg_in->encoding);
    camera_img = cv_img->image;
    image_step = (int)camera_img.step;
    image_elemSize = (int)camera_img.elemSize();
}

void PclEstimationNode::groundPoints_callback(const gbdepth_msgs::msg::GroundPoints::SharedPtr msg_in)
{
    groundArea_avarable = true;
    sample_points = msg_in;
}

void PclEstimationNode::least_squares_calc(vector<vector<float>> data, int N)
{
    float x_ave=0, x2_ave=0, y_ave=0, xy_ave=0, var=0, covar=0;
    for( int i=0; i<N; i++)
    {
        float x_tmp = data[i][0];
        float y_tmp = data[i][1];
        x_ave += x_tmp;
        x2_ave += x_tmp*x_tmp;
        y_ave += y_tmp;
        xy_ave += x_tmp*y_tmp;
    }
    x_ave /= N;
    x2_ave /= N;
    y_ave /= N;
    xy_ave /= N;

    var = x2_ave - x_ave*x_ave;
    covar = xy_ave - x_ave*y_ave;

    SCALE_VALUE = covar/var;
    SHIFT_VALUE = y_ave - SCALE_VALUE*x_ave;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PclEstimationNode)