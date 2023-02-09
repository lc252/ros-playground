#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

ros::Publisher pub;

typedef pcl::FieldComparison<pcl::PointXYZ> FieldComp;

int read_pcd_cb(std::string filename)
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    reader.read(filename, *cloud);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";

    pub.publish(output);

    return 0;
}

int read_obj_cb(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadOBJFile<pcl::PointXYZ>(filename, *object);

    pcl::PassThrough<pcl::PointXYZ> pass;
    // keep everything except bottom
    pass.setInputCloud(object);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 0.5);
    pass.filter(*object);

    // remove front wheels
    pcl::ConditionOr<pcl::PointXYZ>::Ptr front_wheel(new pcl::ConditionOr<pcl::PointXYZ>);
    front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::LT, 0)));
    front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::GT, 0.012)));
    front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::GT,  0.011)));
    front_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::LT, -0.009)));

    // remove back wheels
    pcl::ConditionOr<pcl::PointXYZ>::Ptr back_wheel(new pcl::ConditionOr<pcl::PointXYZ>);
    back_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::LT, 0)));
    back_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("z", pcl::ComparisonOps::GT, 0.012)));
    back_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::GT, -0.061)));
    back_wheel->addComparison(FieldComp::ConstPtr (new FieldComp("y", pcl::ComparisonOps::LT, -0.079)));

    pcl::ConditionalRemoval<pcl::PointXYZ> cond_rem;
    cond_rem.setCondition(front_wheel);
    cond_rem.setInputCloud(object);
    cond_rem.filter(*object);
    cond_rem.setInputCloud(object);
    cond_rem.setCondition(back_wheel);
    cond_rem.setInputCloud(object);
    cond_rem.filter(*object);


    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*object, output);
    output.header.frame_id = "map";

    pub.publish(output);

    return 0;
}

void write_pcd_cb()
{
    ;//pcl::io::;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle n;

    ros::Rate loop_rate(1);

    pub = n.advertise<sensor_msgs::PointCloud2>("raw_points", 1);

    while (ros::ok())
    {
        // read_pcd_cb("/home/fif/lc252/srs-digital-twins/src/pcl_gen/pcd_files/chef.pcd");
        read_obj_cb("/home/fif/lc252/inference-2d-3d/src/object_detection/obj_models/model_car_scaled.obj");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}