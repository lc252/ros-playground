#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

ros::Publisher pub;

enum Links
{
    base_link,
    link_1,
    link_2,
    link_3,
    link_4,
    link_5,
    link_6
};

int read_pcd_cb(std::string filename)
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    reader.read(filename, *cloud);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";

    pub.publish(output);

    return 0;
}

int read_obj_cb(int link_num)
{
    ROS_INFO("%i", link_num);

    std::string link_name;
    float quat_f[4] = {0,0,0,1};
    Eigen::Vector3f tran;
    tran << 0, 0, 0;
    quat_f[0] = 0; quat_f[1] = 0; quat_f[2] = 0; quat_f[3] = 1;
    switch (link_num)
    {
    case Links::base_link:
        link_name = "base_link";
        break;
    case Links::link_1:
        link_name = "link_1";
        break;
    case Links::link_2:
        link_name = "link_2";
        break;
    case Links::link_3:
        link_name = "link_3";
        break;
    case Links::link_4:
        link_name = "link_4";
        break;
    case Links::link_5:
        link_name = "link_5";
        break;
    case Links::link_6:
        link_name = "link_6";
        break;
    }

    ROS_INFO("%s", link_name.c_str());

    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    std::string filename = "/home/lachl/ros-playground/src/pcl_gen/pcd_files/" + link_name + ".pcd";

    pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *object);

    // transform the point cloud to the correct orientation
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf quat(quat_f[3], quat_f[0], quat_f[1], quat_f[2]);    // eigen takes w,x,y,z
    transform_1.block<3,3>(0,0) = quat.toRotationMatrix();
    transform_1.block<3,1>(0,3) = tran;
    pcl::transformPointCloud(*object, *object, transform_1);

    // publish the point cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*object, output);
    output.header.frame_id = link_name;

    pub.publish(output);

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_pcd_publisher");
    ros::NodeHandle n;

    ros::Rate loop_rate(0.5);

    pub = n.advertise<sensor_msgs::PointCloud2>("raw_points", 1);

    while (ros::ok())
    {
        for (int i=0; i<=6; i++)
        {
            read_obj_cb(i);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}