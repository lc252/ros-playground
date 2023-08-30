#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>



typedef pcl::FieldComparison<pcl::PointXYZ> FieldComp;

ros::Publisher pub;
bool saved = 0;

int read_obj_cb(std::string link_name)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
    std::string filename = "/home/lachl/ros-playground/src/pcl_gen/pcd_files/" + link_name + ".obj";

    pcl::io::loadOBJFile<pcl::PointXYZ>(filename, *object);

    // transform the point cloud to the correct orientation
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf quat(0.7071, 0, 0.7071, 0);    // eigen takes w,x,y,z
    transform_1.block<3,3>(0,0) = quat.toRotationMatrix();
    transform_1.block<3,1>(0,3) = Eigen::Vector3f(0, 0, 0);
    pcl::transformPointCloud(*object, *object, transform_1);

    // // filter out points that are too far away
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(object);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-0.175, 0.5);
    // pass.filter(*object);

    // // remove other parts
    // pcl::ConditionOr<pcl::PointXYZ>::Ptr front_wheel(new pcl::ConditionOr<pcl::PointXYZ>);
    // front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::GT, 0.5)));
    // front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::LT, -0.08)));
    // front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::LT,  -0.07)));
    // front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::GT, 0.07)));
    // pcl::ConditionalRemoval<pcl::PointXYZ> cond_rem;
    // cond_rem.setCondition(front_wheel);
    // cond_rem.setInputCloud(object);
    // cond_rem.filter(*object);

    // publish the point cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*object, output);
    output.header.frame_id = "map";

    pub.publish(output);

    // save the transformed pointcloud to pcd format once
    if (!saved)
    {
        std::string pcd_filename = "/home/lachl/ros-playground/src/pcl_gen/pcd_files/" + link_name + ".pcd";
        pcl::io::savePCDFileASCII(pcd_filename, *object);
        saved = 1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle n;

    ros::Rate loop_rate(0.5);

    pub = n.advertise<sensor_msgs::PointCloud2>("raw_points", 1);

    std::string link_name = "link_6";

    while (ros::ok())
    {
        read_obj_cb(link_name);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

