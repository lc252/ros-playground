// tutorial on https://pcl.readthedocs.io/projects/tutorials/en/master/extract_indices.html#extract-indices

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


ros::Publisher pub;


int filter_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2 cloud_filtered_blob;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    
    // Convert go from sensor_msgs to PCL PointCloud2
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl_conversions::toPCL (*input, *cloud);

    // std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

    // First downsample the cloud for simpler processing -> downsampling a plane should still be a plane
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);  // 0.1m, quite coarse
    sor.filter (cloud_filtered_blob);

    // Convert PCLPointCloud2 to PointCloud<T>
    pcl::fromPCLPointCloud2 (cloud_filtered_blob, *cloud_filtered);

    // std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->size();
    // While at least 30% of the original cloud remains
    while (cloud_filtered->size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        
        if (inliers->indices.size () == 0)
        {
            // std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        // cloud plane
        extract.filter (*cloud_p);
        // std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        // get the part of the cloud that was not in the largest planar section
        extract.setNegative (true);
        extract.filter (*cloud_f);
        // make cloud_filtered cloud_f so that the remaining points are processed in the next iteration
        cloud_filtered.swap (cloud_f);

        // convert the output which is a PCL PointCloud<T> to a sensor_msgs PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_p, output);
        pub.publish(output);    // Problem with this at the moment where both point clouds will be sent separately on the same topic
    }

    return (0);
    }

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "extract_indices_example");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("input", 1, filter_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}
