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

    double roll, pitch, yaw;
    tf::Matrix3x3 (r_m).getRPY(roll, pitch, yaw);
    std::cout << roll << " " << pitch << " " << yaw << std::endl;

    // construct a pcl transform
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // translate - x, y, z
    transform.translation() << t_m[0], t_m[1], t_m[2];
    

    // taking the tf base_link->camera_link
    transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())); //yaw
    transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())); //roll
    
    // execute transform
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::transformPointCloud(cloud, transformed_cloud, transform);

    // manual
    // translate - x, y, z
    transform.translation() << 0, 0, 0;
    
    // taking the tf base_link->camera_link, manually tf'ing camera_link->camera_dept_optical_frame
    transform.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(-1.571, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())); //roll

    pcl::transformPointCloud(transformed_cloud, transformed_cloud, transform);

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
        tf_listener.lookupTransform("/base_link", "/camera_depth_optical_frame", ros::Time::now() - delay, camera_transform);
        transform_cloud(camera_transform);
        std::cout << "Cloud sent. " << std::endl;
    }

    
    return 0;
}

