// upon user input, capture the most recent cloud and camera frame transform, and transform the cloud to the global frame

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>


pcl::PointCloud<pcl::PointXYZ> input_cloud;

ros::Publisher pub;


void retrieve_cloud(const sensor_msgs::PointCloud2 cloud2_msg)
{
    pcl::fromROSMsg(cloud2_msg, input_cloud);
}

void transform_cloud(tf::StampedTransform camera_transform)
{
    // get translation and rotation from the transform
    tf::Vector3 t = camera_transform.getOrigin(); 
    tf::Quaternion q = camera_transform.getRotation();

    // console output
    std::cout << "x: " << t.x() << " y: " << t.y() << " z: " << t.z() << std::endl;
    std::cout << "x: " << q.x() << " y: " << q.y() << " z: " << q.z() << " w: " << q.w() << std::endl;    

    // create matrices compatible with pcl transforms
    Eigen::Quaternionf quat(q.w(), q.x(), q.y(), q.z());
    Eigen::Vector3f tran(t.x(), t.y(), t.z());
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

    // Build transformation matrix
    /* Description
    Rotation matrix is 3x3 derived from quaternion, describes a unique rotational pose
    Translation is a 3x1 vector

    Transformation matrix
    | R00 R01 R02 T00 |
    | R10 R11 R12 T01 |
    | R20 R21 R22 T02 |
    |   0   0   0   1 |
    */

    // insert rotation
    transformation_matrix.block<3,3>(0,0) = quat.toRotationMatrix();
    // insert translation
    transformation_matrix.block<3,1>(0,3) = tran;
    // execute transform
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(input_cloud, transformed_cloud, transformation_matrix);

    // create and publish ROS PointCloud2
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(transformed_cloud, output_cloud);
    output_cloud.header.frame_id = "point_map";
    pub.publish(output_cloud);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_cloud");
    ros::NodeHandle nh;

    ros::Rate rate(30);

    // setup cloud subscriber and publisher
    ros::Subscriber sub = nh.subscribe("depth_points", 1, retrieve_cloud);
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
        tf_listener.waitForTransform("/camera", "/map", ros::Time::now() - delay, timeout);
        tf_listener.lookupTransform("/map", "/camera", ros::Time::now() - delay, camera_transform);
        // transform_cloud(camera_transform);
        transform_cloud(camera_transform);
        std::cout << "Cloud sent. " << std::endl;
    }

    
    return 0;
}

