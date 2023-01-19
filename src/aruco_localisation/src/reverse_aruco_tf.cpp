// Adapted from Shakif Turjo's hear_fiducial.py
// take fiducial transforms and apply them to the point cloud.


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>

ros::Publisher pub;

void inverse_transform(fiducial_msgs::FiducialTransformArray msg)
{
    // compute the inverse transform from camera to fiducial

    // first find the transform with the lowest error
    fiducial_msgs::FiducialTransform inverse;
    inverse.image_error = 999;
    for(fiducial_msgs::FiducialTransform transform : msg.transforms)
    {
        if(transform.image_error < inverse.image_error)
        {
            inverse = transform;
        }

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

        Eigen::Quaternionf quat(
            inverse.transform.rotation.w, 
            inverse.transform.rotation.x, 
            inverse.transform.rotation.y, 
            inverse.transform.rotation.z);
        Eigen::Vector3f tran(
            inverse.transform.translation.x, 
            inverse.transform.translation.y, 
            inverse.transform.translation.z);

        // identity matrix
        Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();
        // insert rotation
        tf_matrix.block<3,3>(0,0) = quat.toRotationMatrix();
        // insert translation
        tf_matrix.block<3,1>(0,3) = tran;

        // inverse transformation matrix
        Eigen::Matrix4f inv_tf = tf_matrix.inverse();
        Eigen::Matrix3f m3 = inv_tf.block<3,3>(0,0);
        // retrieve quat and tran for inversed matrix
        Eigen::Quaternionf inv_quat(inv_tf.block<3,3>(0,0));
        Eigen::Vector3f inv_tran(inv_tf.block<3,1>(0,3));

        // Broadcaster
        static tf::TransformBroadcaster br;

        // tf to be broadcasted
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "aruco_" + std::to_string(inverse.fiducial_id);
        msg.child_frame_id = "camera_link";
        // fill rotation values from Eigen
        std::cout << inv_quat.x() << inv_quat.y() << inv_quat.z() << inv_quat.w() << std::endl;
        msg.transform.rotation.x = inv_quat.x();
        msg.transform.rotation.y = inv_quat.y();
        msg.transform.rotation.z = inv_quat.z();
        msg.transform.rotation.w = inv_quat.w();
        // fill translation values from Eigen
        msg.transform.translation.x = inv_tran.x();
        msg.transform.translation.y = inv_tran.y();
        msg.transform.translation.z = inv_tran.z();

        br.sendTransform(msg);
    }

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

