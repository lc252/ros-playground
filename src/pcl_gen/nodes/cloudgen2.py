import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

def random_PointCloud2(n):
    # generate a sensor_msgs.PointCloud2 object with n random points in 1x1m cube
    
    pc2 = PointCloud2(
        height=1,
        width=n
    )

    pc2.header.frame_id = "point_frame"
    pc2.header.stamp = rospy.Time.now()

    # fields - as described by the documentation
    fields = ['x', 'y', 'z' , 'r', 'g', 'b', 'intensity']

    size_float32 = np.dtype(np.float32).itemsize

    # generator for pointfield with:
    #   name = names in fields array
    #   offset = field number * size of data in a field
    #   datatype = float32 --> if different fields have different 
    #       datatypes then the iterator will not work, perhaps 
    #       define the fields in a dict
    #   count = 1 ( 1 object per field)
    pc2.fields = [PointField(name=n, offset=i*size_float32, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(fields)]

    # generate n random points with x,y,z,field_n, ...
    # 1D array with [x1,y1,z1, ... xn,yn,zn]
    points = np.random.rand(n * len(pc2.fields))
    
    # represents the points in a binary blob as float32's
    # float32's take up 32 bits = 4 bytes
    pc2.data = points.astype(np.float32).tobytes()

    # one point has 3 float32, therefore 12 bytes req.
    pc2.point_step = len(pc2.fields) * size_float32
    
    # num data points (1000) * num fields per point (3) * bytes per point (4)
    pc2.row_step = pc2.point_step * n 
    return pc2


def cloudgen():
    # publish a sensor_msgs.PointCloud2 msg to topic pt_cloud_gen @ 1Hz 
    pub = rospy.Publisher("PointCloud2", PointCloud2, queue_size=1)
    rospy.init_node("pt_cloud_gen", anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        pub.publish(random_PointCloud2(1000))

        rate.sleep()
        

if __name__ == '__main__':
    try:
        cloudgen()
    except rospy.ROSInterruptException:
        pass