#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
ros::Publisher pub;

Marker makeArrow(InteractiveMarker &msg)
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

InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg)
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeArrow(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}

void frameCallback(const ros::TimerEvent &)
{
    // take the current marker pose and publish it as the current camera postion
    // this is a way to simulate the camera moving in order to test accumulating pointclouds
    // timer calls this function every frame to update the transform

    // get the marker pose relative to map
    visualization_msgs::InteractiveMarker int_marker;
    server->get("simple_6dof", int_marker);

    // broadcast the transform
    static tf::TransformBroadcaster br;
    tf::Transform t;
    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z));
    t.setRotation(tf::Quaternion(int_marker.pose.orientation.x, int_marker.pose.orientation.y, int_marker.pose.orientation.z, int_marker.pose.orientation.w));
    br.sendTransform(tf::StampedTransform(t, time, "map", "camera_link"));
}

void buttonFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std_msgs::Bool msg;
    switch(feedback->event_type)
    {
        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            // only capture on mouseup so only one event is triggered
            msg.data = true;
            pub.publish(msg);
            break;
        
        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            // depress button
            visualization_msgs::InteractiveMarker button;
            server->get("button", button);
            button.scale = 0.5;
            break;
    }

    server->applyChanges();
}

void make6DofMarker()
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::Vector3 position = tf::Vector3(0, 0, 0);
    tf::Quaternion orientation = tf::Quaternion(0.0, 0.0, 0.0, 1.0);
    tf::pointTFToMsg(position, int_marker.pose.position);
    tf::quaternionTFToMsg(orientation, int_marker.pose.orientation);
    int_marker.scale = 0.2;

    int_marker.name = "simple_6dof";
    int_marker.description = "sim_camera";

    // insert an arrow
    makeBoxControl(int_marker);
    unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    // axis stay fixed with respect to the fixed frame
    control.orientation_mode = InteractiveMarkerControl::FIXED;

    // create the controls for the marker
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
    menu_handler.apply(*server, int_marker.name);
}

void makeButtonMarker()
{
    InteractiveMarker button_marker;
    button_marker.header.frame_id = "map";
    tf::Vector3 position(0, -1, 0);
    tf::pointTFToMsg(position, button_marker.pose.position);
    button_marker.scale = 1;

    button_marker.name = "button";
    button_marker.description = "capture cloud";

    InteractiveMarkerControl control;
    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.name = "button_control";

    Marker marker;
    marker.type = Marker::CUBE;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;

    control.markers.push_back(marker);
    control.always_visible = true;
    button_marker.controls.push_back(control);

    server->insert(button_marker);
    server->setCallback(button_marker.name, &buttonFeedback);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_control");
    ros::NodeHandle n;

    // create a timer to update the published transforms
    ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

    // publisher
    pub = n.advertise<std_msgs::Bool>("capture", 1);

    // create a marker server
    server.reset(new interactive_markers::InteractiveMarkerServer("camera_control", "", false));

    // wait for server to init
    ros::Duration(0.1).sleep();

    // create the interactive marker
    make6DofMarker();
    makeButtonMarker();

    server->applyChanges();

    ros::spin();

    server.reset();
}