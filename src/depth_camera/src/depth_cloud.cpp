// Script for publishing colour mapped pointclouds from an intel realsense camera to a ROS topic
// adapted from https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl-color/rs-pcl-color.cpp

#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>



ros::Publisher pub;
rs2::pipeline p;
rs2::config cfg;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;


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



int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_cloud");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    pub = nh.advertise<sensor_msgs::PointCloud2>("depth_points", 1);

    // configure and start realsense pipeline
    // 1280x720 produces more points but much slower
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    p.start(cfg);

    
    while(ros::ok())
    {
        cloud_cb();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
