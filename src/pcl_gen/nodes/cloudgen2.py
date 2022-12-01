import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


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
        pc2.width = n * 3

        # generate 1000 random points with x,y,z
        # 1D array with [x1,y1,z1, ... xn,yn,zn]
        points = np.random.rand(1000 * 3)
        # represents the points in a binary blob as float32's
        # float32's take up 32 bits = 4 bytes
        pc2.data = points.astype(np.float32).tobytes()

        # one point has 3 float32, therefore 12 bytes req.
        pc2.point_step = 12 # np.dtype(dtype).itemsize * 3
        # only 1 row of 1000*3 points with 4 bytes per point
        pc2.row_step = 12000 

        # fields
        pc2.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        pc2.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        pc2.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))

        pub.publish(pc2)

        rate.sleep()
        

if __name__ == '__main__':
    try:
        cloudgen()
    except rospy.ROSInterruptException:
        pass