#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

int read_cb(std::string filename)
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    reader.read(filename, *cloud);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);

    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle n;

    ros::Rate loop_rate(1);

    ros::Publisher pub;
    pub = n.advertise<sensor_msgs::PointCloud2>("pcd_pointcloud", 1);

    while(ros::ok())
    {
        read_cb("table_scene_lms400.pcd");
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}