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
    fiducial_msgs::FiducialTransform best_tf;
    best_tf.image_error = 999;
    for(fiducial_msgs::FiducialTransform transform : msg.transforms)
    {
        if(transform.image_error < best_tf.image_error)
        {
            best_tf = transform;
        }
    }

    // extract quaternion and translation vector from the received fiducial_msgs::Fiducialransform
    Eigen::Quaternionf quat(
        best_tf.transform.rotation.w, 
        best_tf.transform.rotation.x, 
        best_tf.transform.rotation.y, 
        best_tf.transform.rotation.z);
    Eigen::Vector3f tran(
        best_tf.transform.translation.x, 
        best_tf.transform.translation.y, 
        best_tf.transform.translation.z);

    // Build transformation matrix
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
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "aruco_" + std::to_string(best_tf.fiducial_id);
    tf_msg.child_frame_id = "camera_link";
    // fill rotation values from Eigen
    std::cout << inv_quat.x() << inv_quat.y() << inv_quat.z() << inv_quat.w() << std::endl;
    tf_msg.transform.rotation.x = inv_quat.x();
    tf_msg.transform.rotation.y = inv_quat.y();
    tf_msg.transform.rotation.z = inv_quat.z();
    tf_msg.transform.rotation.w = inv_quat.w();
    // fill translation values from Eigen
    tf_msg.transform.translation.x = inv_tran.x();
    tf_msg.transform.translation.y = inv_tran.y();
    tf_msg.transform.translation.z = inv_tran.z();

    br.sendTransform(tf_msg);

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

