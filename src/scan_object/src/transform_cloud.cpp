// upon user input, capture the most recent cloud and camera frame transform, and transform the cloud to the global frame

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>


pcl::PointCloud<pcl::PointXYZRGB> cloud;

ros::Publisher pub;


void retrieve_cloud(const sensor_msgs::PointCloud2 cloud2_msg)
{
    pcl::fromROSMsg(cloud2_msg, cloud);
}

void transform_cloud(tf::StampedTransform camera_transform)
{
    // get translation and rotation vectors
    tf::Vector3 t_m = camera_transform.getOrigin(); 
    tf::Quaternion r_m = camera_transform.getRotation();

    // construct a pcl transform
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // translate - x, y, z
    transform.translation() << t_m[0], t_m[1], t_m[2];
    
    // rotate - roll, pitch, yaw
    // must convert quarternion values to RPY for pcl tfs
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (r_m[3] * r_m[0] + r_m[1] * r_m[2]);
    double cosr_cosp = 1 - 2 * (r_m[0] * r_m[0] + r_m[1] * r_m[1]);
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double pitch = std::asin(2 * (r_m[3] * r_m[1] - r_m[2] * r_m[0]));
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (r_m[3] * r_m[2] + r_m[0] * r_m[1]);
    double cosy_cosp = 1 - 2 * (r_m[1] * r_m[1] + r_m[2] * r_m[2]);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    // taking the tf base_link->camera_link, manually tf'ing camera_link->camera_dept_optical_frame
    transform.rotate(Eigen::AngleAxisf(roll - 1.571*2, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(pitch + 1.571, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(yaw + 1.571, Eigen::Vector3f::UnitZ()));
    
    // execute transform
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::transformPointCloud(cloud, transformed_cloud, transform);

    // create and publish ROS PointCloud2
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(transformed_cloud, ros_cloud);
    ros_cloud.header.frame_id = "map";
    pub.publish(ros_cloud);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_cloud");
    ros::NodeHandle nh;

    ros::Rate rate(30);

    // setup cloud subscriber and publisher
    ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 1, retrieve_cloud);
    pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 1);    // declared globally

    // setup camera transform listener
    tf::TransformListener tf_listener;
    tf::StampedTransform camera_transform;

    sensor_msgs::PointCloud2 cloud_out;

    while(ros::ok())
    {   
        // wait for enter key
        std::cout << "Enter to capture cloud. ";
        std::cin.get();

        ros::spinOnce();
        
        ros::Duration timeout(1);
        ros::Duration delay(0.01);

        // get the latest camera transform      
        tf_listener.waitForTransform("/camera_link", "/base_link", ros::Time::now() - delay, timeout);
        tf_listener.lookupTransform("/map", "/camera_link", ros::Time::now() - delay, camera_transform);
        transform_cloud(camera_transform);
        std::cout << "Cloud sent. " << std::endl;
    }

    
    return 0;
}

