#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ> cloud_out;


void aggregate_cloud(sensor_msgs::PointCloud2 cloud_in)
{
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    pcl::fromROSMsg(cloud_in, new_cloud);

    // filter new_cloud as aggregating clouds tends to be expensive
    // passthrough to the area of interest
    // voxel downsampling

    cloud_out += new_cloud;

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud_out, ros_cloud);
    ros_cloud.header.frame_id = "point_map";
    pub.publish(ros_cloud);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_clouds");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("transformed_cloud", 1, aggregate_cloud);
    pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);

    ros::spin();
}