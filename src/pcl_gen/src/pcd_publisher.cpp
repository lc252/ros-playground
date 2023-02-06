#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;

int read_cb(std::string filename)
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    reader.read(filename, *cloud);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "point_frame";

    pub.publish(output);

    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle n;

    ros::Rate loop_rate(1);

    pub = n.advertise<sensor_msgs::PointCloud2>("raw_points", 1);

    while(ros::ok())
    {
        read_cb("/home/fif/lc252/srs-digital-twins/src/pcl_gen/pcd_files/chef.pcd");
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}