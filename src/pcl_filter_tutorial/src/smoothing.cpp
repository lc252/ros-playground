#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

ros::Publisher pub;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);


    // Perform the actual filtering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);
    mls.setInputCloud (cloud.makeShared());     // setInputCloud for the MLS filter requires const pcl::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>> &cloud
    mls.setPolynomialOrder (1);     // higher degree takes more to process
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.1);

    mls.process (mls_points);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(mls_points, output);
    pub.publish(output);
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "smoothing_example");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output_smoothed", 1);
    // Spin
    ros::spin();
}
