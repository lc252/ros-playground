// adapted an intel realsense script for ros
// I am hoping to simplify and limit the information streams coming from the camera

#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <std_msgs/Float32.h>

ros::Publisher pub;
rs2::pipeline p;

int depth_cb() // try
{
    rs2::frameset frames = p.wait_for_frames();
    rs2::depth_frame depth = frames.get_depth_frame();
    float w = depth.get_width();
    float h = depth.get_height();

    float centre_dist = depth.get_distance(w/2, h/2);

    // std::cout << "The camera is facing an object " << centre_dist << "m away" << std::endl;
    
    std_msgs::Float32 msg;
    msg.data = centre_dist;
    pub.publish(msg);

    return EXIT_SUCCESS; // 0
}

int main(int argc, char** argv) 
{
    // Initialize ROS
    ros::init(argc, argv, "simple_depth");
    ros::NodeHandle nh;

    // Create a ROS publisher
    pub = nh.advertise<std_msgs::Float32>("depth", 1);

    // publish @ 30hz, tested and 30hz is basically the maximum this thing can handle anyway
    ros::Rate rate(30);

    // start realsense camera pipeline
    p.start();

    // start publish loop
    while(ros::ok())
    {
        depth_cb();
        ros::spinOnce();
        rate.sleep();
    }
}


// catch (const rs2::error & e)
// {
//     std::cerr << "Realsense error call " << e.get_failed_function() << "(" << e.get_failed_args() << ")\n    " << e.what() << std::endl;
//     return EXIT_FAILURE;
// }
// catch (const std::exception & e)
// {
//     std::cerr << e.what() << std::endl;
//     return EXIT_FAILURE;
// }