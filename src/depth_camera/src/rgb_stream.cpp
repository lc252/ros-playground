#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>


// Create a RealSense pipeline, which serves as the container for the processing blocks
rs2::pipeline p;
// Create a configuration for configuring the pipeline with a non default profile
rs2::config cfg;

image_transport::Publisher pub;


void rgb_frame_cb()
{
    // Wait for the next set of frames from the camera   
    rs2::frameset frames = p.wait_for_frames();
    // Get the color frame from the frameset
    rs2::frame color_frame = frames.get_color_frame();
    // Create a cv::Mat object from the color frame
    cv::Mat image = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    
    // Convert the cv::Mat object to a sensor_msgs/Image message
    cv_bridge::CvImage cv_im;

    cv_im.encoding = "bgr8"; // breaks
    cv_im.image = image;
    sensor_msgs::ImagePtr msg = cv_im.toImageMsg();

    // Publish the message
    pub.publish(msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_image_publisher");
    ros::NodeHandle nh;
    // ros::Rate rate(30);

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8);

    // Start the pipeline with the configuration
    p.start(cfg);

    // image publisher
    image_transport::ImageTransport it(nh);
    pub = it.advertise("image", 1);

    while (ros::ok())
    {
        rgb_frame_cb();
        ros::spinOnce();
        // rate.sleep();
    }

    return 0;
}