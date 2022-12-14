#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

using namespace visualization_msgs;


boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;


Marker makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::ARROW;
    marker.scale.x = msg.scale * 0.2 * 5;
    marker.scale.y = msg.scale * 0.05 * 5;
    marker.scale.z = msg.scale * 0.05 * 5;
    marker.color.r = 0;
    marker.color.g = 0.75;
    marker.color.b = 0.75;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void frameCallback(const ros::TimerEvent&)
{
    static tf::TransformBroadcaster br;

    tf::Transform t;

    ros::Time time = ros::Time::now();

    visualization_msgs::InteractiveMarker int_marker;
    server->get("simple_6dof", int_marker);

    t.setOrigin(tf::Vector3(int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z));
    t.setRotation(tf::Quaternion(int_marker.pose.orientation.x, int_marker.pose.orientation.y, int_marker.pose.orientation.z, int_marker.pose.orientation.w));
    br.sendTransform(tf::StampedTransform(t, time, "base_link", "camera_link"));
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    // output change to console
    std::ostringstream s;
    if( feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {
        ROS_INFO_STREAM( s.str() << ": pose changed"
            << "\nposition = "
            << feedback->pose.position.x
            << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z
            << "\norientation = "
            << feedback->pose.orientation.w
            << ", " << feedback->pose.orientation.x
            << ", " << feedback->pose.orientation.y
            << ", " << feedback->pose.orientation.z
            << "\nframe: " << feedback->header.frame_id
            << " time: " << feedback->header.stamp.sec << "sec, "
            << feedback->header.stamp.nsec << " nsec" );
    }

    server->applyChanges();
}

void make6DofMarker()
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::Vector3 position = tf::Vector3(0, 0, 0);
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 0.2;

    int_marker.name = "simple_6dof";
    int_marker.description = "camera";

    // insert a box
    makeBoxControl(int_marker);
    unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    control.orientation_mode = InteractiveMarkerControl::FIXED;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);


    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    menu_handler.apply( *server, int_marker.name );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_controls");
    ros::NodeHandle n;

    // create a timer to update the published transforms
    ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    ros::Duration(0.1).sleep();

    make6DofMarker();

    server->applyChanges();

    ros::spin();

    server.reset();
}