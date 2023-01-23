#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>



// Create a RealSense pipeline, which serves as the container for the processing blocks
rs2::pipeline p;
// Create a configuration for configuring the pipeline with a non default profile
rs2::config cfg;

// image publisher
ros::Publisher pub;


pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_Conversion(const rs2::points& points){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    // PCL Cloud Object Configuration
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    // auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        // map xyz, alter to fit with the world xyz frame
        cloud->points[i].x = Vertex[i].z;
        cloud->points[i].y = -Vertex[i].x;
        cloud->points[i].z = -Vertex[i].y;

    }
    
    // return pcl pointcloud
    return cloud;
}

int cloud_cb()
{
    rs2::pointcloud pc;
    rs2::points points;

    auto frames = p.wait_for_frames();
    auto depth = frames.get_depth_frame();

    points = pc.calculate(depth);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PCL_Conversion(points);

    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*cloud, output_cloud);

    output_cloud.header.frame_id = "camera";
    pub.publish(output_cloud);

    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_stream");
    ros::NodeHandle nh;
    //ros::Rate rate(30);


    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // image publisher
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("image", 1);

    // cloud publisher
    pub = nh.advertise<sensor_msgs::PointCloud2>("depth_points", 1);

    // Start the pipeline with the configuration
    p.start(cfg);

    while (ros::ok())
    {
        // RGB
        // Wait for the next set of frames from the camera   
        rs2::frameset frames = p.wait_for_frames();
        // Get the color frame from the frameset
        rs2::frame color_frame = frames.get_color_frame();
        // Create a cv::Mat object from the color frame
        cv::Mat image = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert the cv::Mat object to a sensor_msgs/Image message
        cv_bridge::CvImage cv_im;

        cv_im.encoding = "bgr8"; // breaks
        cv_im.image = image;
        sensor_msgs::ImagePtr msg = cv_im.toImageMsg();

        // Publish the message
        image_pub.publish(msg);

        cloud_cb();

        ros::spinOnce();
        //rate.sleep();
    }

    return 0;
}