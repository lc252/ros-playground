# converted cpp script into python to see if I could do it, and make sure it still works

import sys
import rospy
from visualization_msgs.msg import Marker

def basic_shapes():
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.init_node('basic_shapes', anonymous=True)
    rate = rospy.Rate(1)

    shape = Marker.CUBE
    shapes = [Marker.CUBE, Marker.SPHERE, Marker.CYLINDER, Marker.ARROW]

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "my_frame"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "basic_shapes"
        marker.id = 0

        marker.type = shape

        marker.action = Marker.ADD

        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.r = 0
        marker.color.b = 1
        marker.color.g = 0
        marker.color.a = 1

        marker.lifetime = rospy.Duration()

        while pub.get_num_connections() < 1:
            if rospy.is_shutdown():
                return
            rospy.logwarn_once("Please create a subscriber to the marker")
        pub.publish(marker)

        # cycle shapes: since shapes are enumerated 0,1,2,3 for
        # arrow, cube, sphere, cylinder
        # cube which has position 0 in list "shapes" points to postion 1 and so on
        shape = shapes[shape]

        rate.sleep()

if __name__ == '__main__':
    try:
        basic_shapes()
    except rospy.ROSInterruptException:
        pass