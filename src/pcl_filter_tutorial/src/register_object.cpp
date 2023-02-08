#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>


// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;

// Publishers
ros::Publisher object_pub;
ros::Publisher scene_pub;
ros::Publisher object_aligned_pub;



void register_object_cb(sensor_msgs::PointCloud2 input)
{
    // Point clouds
    PointCloudT::Ptr object(new PointCloudT);
    PointCloudT::Ptr object_aligned(new PointCloudT);
    PointCloudT::Ptr scene(new PointCloudT);
    PointCloudT::Ptr clip(new PointCloudT);
    FeatureCloudT::Ptr object_features(new FeatureCloudT);
    FeatureCloudT::Ptr scene_features(new FeatureCloudT);


    // load scene
    pcl::fromROSMsg(input, *clip);

    // simulate finding an ROI by clipping 100px on left right top bottom
    pcl::console::print_highlight("load scene...\n");
    for (int i=99; i<(clip->height-100); i++)
    {
        for (int j=99; j<(clip->width-100); j++)
        {
            
            if (!std::isnan(clip->at(j,i).x))
            {
                scene->push_back(clip->at(j,i));    
            }
            else 
            {
                ;//std::cout << clip->at(j,i).x << " ";
            }
        }
    }

    // // remove NANs from scene
    // boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    // pcl::removeNaNFromPointCloud(*scene, *indices);
    // pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // extract.setInputCloud(scene);
    // extract.setIndices(indices);
    // extract.setNegative(true);
    // extract.filter(*scene);
    // load object
    pcl::io::loadOBJFile<PointNT>("/home/fif/lc252/inference-2d-3d/src/object_detection/obj_models/hp_mouse_scaled.obj", *object);

    // Downsample
    pcl::console::print_highlight("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 0.005f;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(object);
    grid.filter(*object);
    // too much detail lost in filtering
    grid.setLeafSize(0.002, 0.002, 0.002);
    grid.setInputCloud(scene);
    grid.filter(*scene);

    // Estimate normals for object and scene
    pcl::console::print_highlight("Normal Estimation...\n");
    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setRadiusSearch(0.01);
    nest.setInputCloud(object);
    nest.compute(*object);
    nest.setRadiusSearch(0.01);
    nest.setInputCloud(scene);
    nest.compute(*scene);

    // Estimate features
    pcl::console::print_highlight("features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch(0.025);
    fest.setInputCloud(object);
    fest.setInputNormals(object);
    fest.compute(*object_features);
    fest.setInputCloud(scene);
    fest.setInputNormals(scene);
    fest.compute(*scene_features);

    // Perform alignment
    pcl::console::print_highlight("Alignment...\n");
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
    align.align(*object_aligned);

    // get the transform Eigen
    pcl::console::print_highlight("Send transform...\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation();
    transformation = transformation.inverse();  // reverse from object->scene to scene->object 
    Eigen::Vector3f t(transformation.block<3,1>(0,3));
    Eigen::Quaternionf q(transformation.block<3,3>(0,0));
    // create transform TF
    geometry_msgs::TransformStamped object_alignment_tf;
    object_alignment_tf.header.stamp = ros::Time::now();
    object_alignment_tf.header.frame_id = "camera_color_optical_frame";
    object_alignment_tf.child_frame_id = "aligned_object";
    object_alignment_tf.transform.translation.x = t.x();
    object_alignment_tf.transform.translation.y = t.y();
    object_alignment_tf.transform.translation.z = t.z();
    object_alignment_tf.transform.rotation.x = q.x();
    object_alignment_tf.transform.rotation.y = q.y();
    object_alignment_tf.transform.rotation.z = q.z();
    object_alignment_tf.transform.rotation.w = q.w();
    // broadcast
    static tf::TransformBroadcaster br;
    br.sendTransform(object_alignment_tf);

    // Create ros msg clouds
    pcl::console::print_highlight("Cloud...\n");
    sensor_msgs::PointCloud2 ros_object, ros_scene, ros_object_aligned;
    pcl::toROSMsg(*object, ros_object);
    ros_object.header.frame_id = "map";
    pcl::toROSMsg(*scene, ros_scene);
    ros_scene.header.frame_id = "map";
    pcl::toROSMsg(*object_aligned, ros_object_aligned);
    ros_object_aligned.header.frame_id = "map";
    // Publish clouds
    object_pub.publish(ros_object);
    scene_pub.publish(ros_scene);
    object_aligned_pub.publish(ros_object_aligned);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "registration_node");
    ros::NodeHandle nh;

    // setup pubs
    object_pub = nh.advertise<sensor_msgs::PointCloud2>("object", 1);
    scene_pub = nh.advertise<sensor_msgs::PointCloud2>("scene", 1);
    object_aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("object_aligned", 1);

    // setup sub
    ros::Subscriber sub = nh.subscribe("camera/depth_registered/points", 1, register_object_cb);
    
    ros::spin();

    return (0);
}
