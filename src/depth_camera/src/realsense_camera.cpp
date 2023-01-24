#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/Transform.h>


class RealSense_Camera
{

public:
    RealSense_Camera()
    {
        // setup streams
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16);
        // start streaming
        p.start(cfg);

        // publish the static tf tree
        publish_static_tf();

        ROS_INFO("Camera Streams Started.");
    }

    void get_frames()
    {
        frames = p.wait_for_frames();
    }

    sensor_msgs::ImagePtr get_rgb()
    {
        rs2::frame color_frame = frames.get_color_frame();
        // Create a cv::Mat object from the color frame
        cv::Mat mat = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        
        // Convert the cv::Mat object to a sensor_msgs/Image message
        cv_bridge::CvImage cv_im;

        cv_im.encoding = "bgr8"; // breaks
        cv_im.image = mat;
        sensor_msgs::ImagePtr ros_image = cv_im.toImageMsg();
        return ros_image;
    }

    sensor_msgs::PointCloud2 get_cloud()
    {
        rs2::pointcloud pc;
        rs2::points points;

        rs2::frame depth_frame = frames.get_depth_frame();

        points = pc.calculate(depth_frame);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PCL_Conversion(points);

        // convert to ROS pointcloud
        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*cloud, output_cloud);
        output_cloud.header.frame_id = "depth_frame";

        return output_cloud;
    }

private: 
    // pipeline and configuration for camera stream
    rs2::pipeline p;
    rs2::config cfg;

    // encapsulate frames
    rs2::frameset frames;

    // static tf publisher
    tf2_ros::StaticTransformBroadcaster br;

    pcl::PointCloud<pcl::PointXYZ>::Ptr PCL_Conversion(const rs2::points& points)
    {
        // pcl_cloud is a pointer to pcl cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // PCL Cloud Object Configuration
        // Convert data captured from Realsense camera to Point Cloud
        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        
        pcl_cloud->width  = static_cast<uint32_t>(sp.width());   
        pcl_cloud->height = static_cast<uint32_t>(sp.height());
        pcl_cloud->is_dense = false;
        pcl_cloud->points.resize(points.size());

        auto Vertex = points.get_vertices();

        // Iterating through all points and setting XYZ coordinates
        for (int i = 0; i < points.size(); i++)
        {   
            // map xyz, alter to fit with the world xyz frame
            pcl_cloud->points[i].x = Vertex[i].z;
            pcl_cloud->points[i].y = -Vertex[i].x;
            pcl_cloud->points[i].z = -Vertex[i].y;

        }
        
        return pcl_cloud;
    }

    void publish_static_tf()
    {
        get_frames();

        // The base frame for the camera is analogous with the depth frame
        rs2::stream_profile base_frame = frames.get_depth_frame().get_profile();
        rs2::stream_profile color_frame = frames.get_color_frame().get_profile();
        // get the tf from base to color frame. Realsense terminology is backwards.
        rs2_extrinsics ext = color_frame.get_extrinsics_to(base_frame);

        
        Eigen::Matrix3f m;
        m << ext.rotation[0], ext.rotation[3], ext.rotation[6],
             ext.rotation[1], ext.rotation[4], ext.rotation[7],
             ext.rotation[2], ext.rotation[5], ext.rotation[8];

        Eigen::Quaternionf q(m);

        geometry_msgs::TransformStamped tf;
        tf.header.frame_id = "depth_frame";
        tf.child_frame_id = "color_frame";
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf.transform.translation.x = ext.translation[0];
        tf.transform.translation.y = ext.translation[1];
        tf.transform.translation.z = ext.translation[2];
        
        br.sendTransform(tf);

    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_stream");
    ros::NodeHandle nh;

    // cloud publisher
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("camera/depth", 1);
    // image publisher
    image_transport::ImageTransport it(nh);
    image_transport::Publisher im_pub = it.advertise("camera/image", 1);


    // Camera
    RealSense_Camera cam;

    // Publish the static tf's
    cam.get_frames();

    while (ros::ok())    
    {
        // get the latest frames
        cam.get_frames();

        // get and publish the colour image
        im_pub.publish(cam.get_rgb());

        // get the point cloud
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = cam.get_cloud();
        cloud_pub.publish(cam.get_cloud());

        // there is no ros::Rate() specified, therefore the speed is limited by the frame rate of the cameras
        ros::spinOnce();
    }

    return 0;
}