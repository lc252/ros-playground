import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

def points_and_lines():
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.init_node("points_and_lines")
    rate = rospy.Rate(30)

    f = 0.0

    while not rospy.is_shutdown():
        points = Marker()
        line_strip = Marker()
        line_list = Marker()

        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "my_frame"
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rospy.Time.now()
        points.ns = line_strip.ns = line_list.ns = "points_and_lines"
        points.action = line_strip.action = line_list.action = Marker.ADD
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0

        points.id = 0
        line_strip.id = 1
        line_list.id = 2

        points.type = Marker.POINTS
        line_strip.type = Marker.LINE_STRIP
        line_list.type = Marker.LINE_LIST

        points.scale.x = points.scale.y = 0.2
        line_strip.scale.x = 0.1
        line_list.scale.x = 0.1

        points.color.g = 1
        points.color.a = 1

        line_strip.color.b = 1
        line_strip.color.a = 1

        line_list.color.r = 1
        line_list.color.a = 1

        for i in range(100):
            y = 5 * np.sin(f + i / 100 * 2 * np.pi)
            z = 5 * np.cos(f + i / 100 * 2 * np.pi)

            p = Point()
            p.x = i - 50
            p.y = y
            p.z = z

            points.points.append(p)
            line_strip.points.append(p)
            line_list.points.append(p)
            p.z += 1
            line_list.points.append(p)
        
        pub.publish(points)
        pub.publish(line_strip)
        pub.publish(line_list)

        rate.sleep()

        f += 0.04

        

if __name__ == '__main__':
    try:
        points_and_lines()
    except rospy.ROSInterruptException:
        pass
