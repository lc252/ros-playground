#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Publisher pub;
rs2::pipeline p;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl_ptr rs_points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();    
    cloud->is_dense = false;    
    cloud->points.resize(points.size()); 
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    return cloud;
}

int cloud_cb()
{
    rs2::pointcloud pc;
    rs2::points points;

    auto frames = p.wait_for_frames();
    auto depth = frames.get_depth_frame();

    points = pc.calculate(depth);
    auto pcl_cloud = rs_points_to_pcl(points);

    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*pcl_cloud, output_cloud);

    output_cloud.header.frame_id = "point_frame";
    pub.publish(output_cloud);

    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_camera_pc");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    pub = nh.advertise<sensor_msgs::PointCloud2>("rs_points", 1);

    p.start();
    
    while(ros::ok())
    {
        cloud_cb();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
