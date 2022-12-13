#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <iostream>

int main(int argc, char* argv[]) try
{
    rs2::pipeline p;
    p.start();
    while (1)
    {
        rs2::frameset frames = p.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        float w = depth.get_width();
        float h = depth.get_height();

        float centre_dist = depth.get_distance(w/2, h/2);

        std::cout << "The camera is facing an object " << centre_dist << "m away" << std::endl;
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "Realsense error call " << e.get_failed_function() << "(" << e.get_failed_args() << ")\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}