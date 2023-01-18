// Adapted from Shakif Turjo's hear_fiducial.py
// take fiducial transforms and apply them to the point cloud.


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <fiducial_msgs/FiducialTransformArray.h>

ros::Publisher pub;

void inverse_transform(fiducial_msgs::FiducialTransformArray transforms)
{
    // compute the inverse transform from camera to fiducial
    for( fiducial_msgs::FiducialTransform);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_cloud");
    ros::NodeHandle nh;

    ros::Rate rate(30);

    // When a fiducial transform is published, callback to inverse it
    ros::Subscriber cloud_sub = nh.subscribe("fiducial_transforms", 1, inverse_transform);

    ros::spin();
        
    return 0;
}

