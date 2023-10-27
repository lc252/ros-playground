#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

ros::Publisher pub;

int read_pcd_cb(std::string link_name)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    std::string filename = "/home/lachl/ros-playground/src/pcl_gen/pcd_files/" + link_name + ".pcd";

    pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *object);

    // transform the point cloud to the correct orientation
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf quat(1, 0, 0, 0);    // eigen takes w,x,y,z
    Eigen::Vector3f tran(0, 0, 0);
    transform_1.block<3,3>(0,0) = quat.toRotationMatrix();
    transform_1.block<3,1>(0,3) = tran;
    pcl::transformPointCloud(*object, *object, transform_1);

    // publish the point cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*object, output);
    output.header.frame_id = "map";

    pub.publish(output);

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle n;

    ros::Rate loop_rate(0.5);

    pub = n.advertise<sensor_msgs::PointCloud2>("raw_points", 1);

    while (ros::ok())
    {
        read_pcd_cb("link_1");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}