#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ> cloud_out;


void aggregate_cloud(sensor_msgs::PointCloud2 cloud_in)
{
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    pcl::fromROSMsg(cloud_in, new_cloud);

    // literally add lol
    cloud_out += new_cloud;

    // filter aggregated cloud
    /* Filters
    Passthrough:
         - reduce the search size to the region of interest
         - ignore points in the far field with poorer accuracy
    Voxel:
         - downsample the number of points for processing
         - averaging samples in space reduces noise
         - average overlapping points from multiple scans
         - voxel size should be close to the sensors resolution
            so that precision is maintained but overlapping points
            are averaged.
    Outlier removal:
         - radial outliers
         - statistical outliers
         - possible but maybe not necessary
    */

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