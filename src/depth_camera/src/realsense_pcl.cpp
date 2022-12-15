#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Publisher pub;
rs2::pipeline p;
rs2::config cfg;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// realsense -> PCL functions from https://github.com/IntelRealSense/librealsense/blob/master/wrappers/pcl/pcl-color/rs-pcl-color.cpp
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    cloud_pointer cloud(new point_cloud);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

    }
    
   return cloud; // PCL RGB Point Cloud generated
}

int cloud_cb()
{
    rs2::pointcloud pc;
    rs2::points points;

    auto frames = p.wait_for_frames();
    auto depth = frames.get_depth_frame();
    auto rgb = frames.get_color_frame();

    pc.map_to(rgb);
    points = pc.calculate(depth);
    auto pcl_cloud = rs_points_to_pcl(points);

    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*pcl_cloud, output_cloud);

    output_cloud.header.frame_id = "point_frame";
    pub.publish(output_cloud);

    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_camera_pc");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    pub = nh.advertise<sensor_msgs::PointCloud2>("rs_points", 1);

    // configure and start realsense pipeline
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    p.start(cfg);

    
    while(ros::ok())
    {
        cloud_cb();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
