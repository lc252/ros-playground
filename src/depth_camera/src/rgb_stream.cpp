#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_image_publisher");
    ros::NodeHandle nh;

    // Create a RealSense pipeline, which serves as the container for the processing blocks
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    // Start the pipeline with the configuration
    pipe.start(cfg);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image", 1);

    while (ros::ok())
    {
        // Wait for the next set of frames from the camera
        rs2::frameset frames = pipe.wait_for_frames();
        // Get the color frame from the frameset
        rs2::frame color_frame = frames.get_color_frame();
        // Create a cv::Mat object from the color frame
        cv::Mat image = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert the cv::Mat object to a sensor_msgs/Image message
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::CvImage::create();
        cv_ptr->encoding = "bgr8";
        cv_ptr->image = image;
        sensor_msgs::ImagePtr msg = cv_ptr->toImageMsg();

        // Publish the message
        pub.publish(msg);

        ros::spinOnce();
    }

    return 0;
}