// A simple example of transforming pointclouds and depth image.

// This project is based on OpenCV (Open Source Computer Vision Library) and is released under the same license.

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chrono>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>

cv::Mat K_input;
ros::Publisher pub_pcd;

void depthToxyzrgb(const cv::Mat& in_depth, const cv::Mat& rgb, const cv::Mat_<float>& K, cv::Mat& points3d)
{
    ROS_ASSERT(K.cols == 3 && K.rows == 3 &&  K.depth()==CV_32F);
    ROS_ASSERT(in_depth.type() == CV_16UC1);
    ROS_ASSERT(rgb.type() == CV_8UC3);

    const float inv_fx = 1.0f / K(0, 0);
    const float inv_fy = 1.0f / K(1, 1);
    const float ox = K(0, 2);
    const float oy = K(1, 2);

    cv::Mat_<float> z_mat;
    in_depth.convertTo(z_mat, CV_32F, 0.001); 



    points3d.create(in_depth.size(), CV_MAKETYPE(CV_32F, 4));



    cv::Mat_<float> x_cache(1, in_depth.cols), y_cache(in_depth.rows, 1);
    float* x_cache_ptr = x_cache[0], *y_cache_ptr = y_cache[0];
    for (int x = 0; x < in_depth.cols; ++x, ++x_cache_ptr)
        *x_cache_ptr = (x - ox) * inv_fx;
    for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
        *y_cache_ptr = (y - oy) * inv_fy;
        y_cache_ptr = y_cache[0];


    for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
    {
        cv::Vec<float, 4>* point = points3d.ptr<cv::Vec<float, 4> >(y);
        const cv::Vec3b* rgb_ptr = rgb.ptr<cv::Vec3b>(y);

        const float* x_cache_ptr_end = x_cache[0] + in_depth.cols;
        const float* depth = z_mat[y];
        
        for (x_cache_ptr = x_cache[0]; x_cache_ptr != x_cache_ptr_end; ++x_cache_ptr, ++point, ++depth, ++rgb_ptr)
        {
            float z = *depth;
            (*point)[0] = (*x_cache_ptr) * z;
            (*point)[1] = (*y_cache_ptr) * z;
            (*point)[2] = z;   

            uint8_t b = (*rgb_ptr)[0]; 
            uint8_t g = (*rgb_ptr)[1];
            uint8_t r = (*rgb_ptr)[2];
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b );
            float *rgb_float = reinterpret_cast<float*>(&rgb);

            (*point)[3] = *rgb_float;
            
        }
    }
}

void depthToxyz(const cv::Mat& in_depth, const cv::Mat_<float>& K, cv::Mat& points3d)
{
    ROS_ASSERT(K.cols == 3 && K.rows == 3 &&  K.depth()==CV_32F);
    ROS_ASSERT(in_depth.type() == CV_16UC1);
    
    const float inv_fx = 1.0f / K(0, 0);
    const float inv_fy = 1.0f / K(1, 1);
    const float ox = K(0, 2);
    const float oy = K(1, 2);
    cv::Mat_<float> z_mat;
    in_depth.convertTo(z_mat, CV_32F, 0.001); 

    points3d.create(in_depth.size(), CV_MAKETYPE(CV_32F, 3));

    cv::Mat_<float> x_cache(1, in_depth.cols), y_cache(in_depth.rows, 1);
    float* x_cache_ptr = x_cache[0], *y_cache_ptr = y_cache[0];
    for (int x = 0; x < in_depth.cols; ++x, ++x_cache_ptr)
        *x_cache_ptr = (x - ox) * inv_fx;
    for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
        *y_cache_ptr = (y - oy) * inv_fy;
        y_cache_ptr = y_cache[0];


    for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
    {
        cv::Vec<float, 3>* point = points3d.ptr<cv::Vec<float, 3> >(y);
        
        const float* x_cache_ptr_end = x_cache[0] + in_depth.cols;
        const float* depth = z_mat[y];
        
        for (x_cache_ptr = x_cache[0]; x_cache_ptr != x_cache_ptr_end; ++x_cache_ptr, ++point, ++depth)
        {
            float z = *depth;
            (*point)[0] = (*x_cache_ptr) * z;
            (*point)[1] = (*y_cache_ptr) * z;
            (*point)[2] = z;   
        }
    }
}

void pubPoincloud(ros::Publisher& publisher, const cv::Mat& pcd, const double t)
{
    if (pcd.empty())
    {
        sensor_msgs::PointCloud2 cloudMsg;
        cloudMsg.header.frame_id = "base_link";
        cloudMsg.header.stamp = ros::Time(t);
        publisher.publish(cloudMsg);
        return;
    }

    sensor_msgs::PointCloud2 cloudMsg;
    cloudMsg.header.frame_id = "base_link";
    cloudMsg.header.stamp = ros::Time(t);

    cloudMsg.height = pcd.rows; // 无组织点云的高度为1
    cloudMsg.width = pcd.cols; // 点云中点的总数
    cloudMsg.is_dense = false; // 点云中可能包含NaN或Inf
    sensor_msgs::PointCloud2Modifier modifier(cloudMsg);

    if(pcd.type() == CV_32FC3)
    {

        modifier.setPointCloud2FieldsByString(1, "xyz");

        cloudMsg.data.resize(pcd.rows * pcd.cols * sizeof(float) * 3); // 3个浮点数（XYZ）每个点
        cloudMsg.point_step = sizeof(float) * 3; // 每个点的大小
        cloudMsg.row_step = cloudMsg.point_step * pcd.cols; // 每一行的大小

        const float* mat_data = (const float*)pcd.data;
        std::memcpy(cloudMsg.data.data(), mat_data, cloudMsg.data.size());
    }
    else if (pcd.type() == CV_MAKETYPE(CV_32F, 4))
    {

        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::PointField::FLOAT32);

        cloudMsg.data.resize(pcd.rows * pcd.cols * sizeof(float) * 4); // 3个浮点数（XYZ）每个点
        cloudMsg.point_step = sizeof(float) * 4; // 每个点的大小
        cloudMsg.row_step = cloudMsg.point_step * pcd.cols; // 每一行的大小

        const float* mat_data = (const float*)pcd.data;
        std::memcpy(cloudMsg.data.data(), mat_data, cloudMsg.data.size());

    }

    publisher.publish(cloudMsg);
}

void callback(const sensor_msgs::ImageConstPtr& rgb_image_msg, const sensor_msgs::ImageConstPtr& depth_image_msg) 
{
    
    //load image
    cv::Mat rgb_image, depth_image;
    rgb_image = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::BGR8)->image;
    depth_image = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

    auto start = std::chrono::high_resolution_clock::now();


    // it will be faster if you integrate it in a nodelet, or enable o3 optimization. 
    // But we found it is incompatible with multi-threading, we are working on it.


    // transform depth image to cvmat rgb pointcloud
    cv::Mat pcd;
    depthToxyzrgb(depth_image, rgb_image, K_input, pcd);         //a modified version, for xyzrgb
    // cv::rgbd::depthTo3d(depth_image, K_input, pcd);              //opencv official version, for xyz
    // depthToxyz(depth_image, K_input, pcd);                          // follow the official version. You can change it if you want additional channels



    // Do something.
    // You can refer to https://docs.opencv.org/4.x/d2/d3a/group__rgbd.html.



    // transform cvmat rgb pointcloud to ros pointcloud2. If you only want to visualize the rgb pcd, rtabmap is a better choice, see our launch file. It is optimized and downsampled. 
    // this function is used for customized visualization.
    pubPoincloud(pub_pcd, pcd,rgb_image_msg->header.stamp.toSec());   //Now it depends on the channels of input cvmat. 3:XYZ   4:XYZRGB


    //check the costing time on your computer
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    ROS_WARN("it took %llu milliseconds to complete.", static_cast<unsigned long long>(duration));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depthandpcd_node");
    ros::NodeHandle n("~");

    K_input = (cv::Mat_<float>(3,3) << 675.585510, 0, 492.865662, 0, 676.195007, 269.670898, 0, 0, 1);
    pub_pcd = n.advertise<sensor_msgs::PointCloud2>("pub_pcd", 100);

    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, "/camera/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, "/camera/aligned_depth_to_color/image_raw", 100);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image, sub_depth);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}