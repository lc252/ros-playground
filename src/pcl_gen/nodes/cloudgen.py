import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32


def random_Point32():
    # return a Point32 with random xyz

    point = Point32()
    # get 3 random values 0 <= x,y,z <= 1
    point.x = np.random.rand()
    point.y = np.random.rand()
    point.z = np.random.rand()
    return point

def cloudgen():
    pub = rospy.Publisher("raw_points", PointCloud, queue_size=1)
    rospy.init_node("pt_cloud_gen", anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pc = PointCloud()
        pc.header.frame_id = "point_frame"
        pc.header.stamp = rospy.Time.now()

        # change n to define the number of points
        n = 1000
        # generate np array with 1 * n random point32's
        pc.points = [random_Point32() for _ in range(n)]
        
        pub.publish(pc)

        rate.sleep()
        

if __name__ == '__main__':
    try:
        cloudgen()
    except rospy.ROSInterruptException:
        pass