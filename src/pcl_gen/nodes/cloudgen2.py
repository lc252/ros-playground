import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2


def cloudgen():
    # publish a sensor_msgs.PointCloud2 msg to topic pt_cloud_gen @ 1Hz 
    pub = rospy.Publisher("PointCloud2", PointCloud2, queue_size=1)
    rospy.init_node("pt_cloud_gen", anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        pc2 = PointCloud2()
        pc2.header.frame_id = "point_frame"
        pc2.header.stamp = rospy.Time.now()

        # change n to define the number of points
        n = 1000

        pc2.height = 1
        pc2.width = n

        pc2.point_step = 12
        
        pub.publish(pc2)

        rate.sleep()
        

if __name__ == '__main__':
    try:
        cloudgen()
    except rospy.ROSInterruptException:
        pass