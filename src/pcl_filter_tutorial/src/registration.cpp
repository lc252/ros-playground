
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
int main(int argc, char** argv)
{
    ros::init(argc, argv, "registration_node");
    // Point clouds
    PointCloudT::Ptr object(new PointCloudT);
    PointCloudT::Ptr object_aligned(new PointCloudT);
    PointCloudT::Ptr scene(new PointCloudT);
    FeatureCloudT::Ptr object_features(new FeatureCloudT);
    FeatureCloudT::Ptr scene_features(new FeatureCloudT);

    // Load object and scene
    pcl::console::print_highlight("Loading point clouds...\n");
    pcl::io::loadOBJFile<PointNT>("/home/fif/lc252/srs-digital-twins/src/pcl_filter_tutorial/obj_files/chef.obj", *object);
    //pcl::io::loadPCDFile<PointNT>("/home/fif/lc252/srs-digital-twins/src/pcl_gen/pcd_files/chef.pcd", *object);
    pcl::io::loadPCDFile<PointNT>("/home/fif/lc252/srs-digital-twins/src/pcl_gen/pcd_files/rs1.pcd", *scene);

    // Downsample
    pcl::console::print_highlight("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 0.005f;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(object);
    grid.filter(*object);
    grid.setInputCloud(scene);
    grid.filter(*scene);

    // Estimate normals for object and scene
    pcl::console::print_highlight("Estimating object and scene normals...\n");
    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setRadiusSearch(0.01);
    nest.setInputCloud(object);
    nest.compute(*object);
    nest.setInputCloud(scene);
    nest.compute(*scene);

    // Estimate features
    pcl::console::print_highlight("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch(0.025);
    fest.setInputCloud(object);
    fest.setInputNormals(object);
    fest.compute(*object_features);
    fest.setInputCloud(scene);
    fest.setInputNormals(scene);
    fest.compute(*scene_features);

    // Perform alignment
    pcl::console::print_highlight("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
    align.setInputSource(object);
    align.setSourceFeatures(object_features);
    align.setInputTarget(scene);
    align.setTargetFeatures(scene_features);
    align.setMaximumIterations(50000);               // Number of RANSAC iterations
    align.setNumberOfSamples(3);                     // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness(5);            // Number of nearest features to use
    align.setSimilarityThreshold(0.95f);              // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
    align.setInlierFraction(0.25f);                  // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("Alignment");
        align.align(*object_aligned);
    }


    // Print results
    printf("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation();
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
    pcl::console::print_info("\n");
    pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

    //     |  0.005 -0.700 -0.714 | 
    // R = | -0.991 -0.101  0.092 | 
    //     | -0.136  0.707 -0.694 | 

    // t = < -0.499, 0.108, 0.290 >

    // the object transformed
    PointCloudT::Ptr tf_object(new PointCloudT);
    // execte transform
    pcl::transformPointCloud(*object, *tf_object, transformation);

    sensor_msgs::PointCloud2 ros_object, ros_scene, ros_tf_object;
    pcl::toROSMsg(*object, ros_object);
    ros_object.header.frame_id = "map";
    pcl::toROSMsg(*scene, ros_scene);
    ros_scene.header.frame_id = "map";
    pcl::toROSMsg(*tf_object, ros_tf_object);
    ros_tf_object.header.frame_id = "map";

    ros::NodeHandle nh;
    ros::Publisher object_pub = nh.advertise<sensor_msgs::PointCloud2>("object", 1);
    ros::Publisher scene_pub = nh.advertise<sensor_msgs::PointCloud2>("scene", 1);
    ros::Publisher tf_object_pub = nh.advertise<sensor_msgs::PointCloud2>("tf_object", 1);

    ros::Rate r(1);

    while (ros::ok())
    {
        object_pub.publish(ros_object);
        scene_pub.publish(ros_scene);
        tf_object_pub.publish(ros_tf_object);
        r.sleep();
        ros::spinOnce();
    }

    return (0);
}
