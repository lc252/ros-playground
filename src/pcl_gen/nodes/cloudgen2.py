import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import time



def random_PointCloud2(n):
    # generate a sensor_msgs.PointCloud2 object with n random points in 1x1m cube
    
    pc2 = PointCloud2(
        height=1,
        width=n
    )

    pc2.header.frame_id = "point_frame"
    pc2.header.stamp = rospy.Time.now()

    # fields - as described by the documentation
    fields = ['x', 'y', 'z', 'r', 'g', 'b', 'intensity']

    size_float32 = np.dtype(np.float32).itemsize

    # generator for pointfield with:
    #   name = names in fields array
    #   offset = field number * size of data in a field
    #   datatype = float32 --> if different fields have different 
    #       datatypes then the iterator will not work, perhaps 
    #       define the fields in a dict
    #   count = 1 ( 1 object per field)
    pc2.fields = [PointField(name=name, offset=i*size_float32, datatype=PointField.FLOAT32, count=1) for i, name in enumerate(fields)]

    # generate n random points with x,y,z,field_n, ...
    # 1D array with [x1,y1,z1, ... xn,yn,zn]
    points = np.random.rand(n * len(pc2.fields))
    
    # represents the points in a binary blob as float32's
    # float32's take up 32 bits = 4 bytes
    pc2.data = points.astype(np.float32).tobytes()

    # one point has 3 float32, therefore 12 bytes req.
    pc2.point_step = len(pc2.fields) * size_float32
    
    # num data points (1000) * num fields per point (3)
    pc2.row_step = pc2.point_step * n 
    return pc2


def magic_carpet(n):
    # generate a sensor_msgs.PointCloud2 object that moves as a carpet
    
    pc2 = PointCloud2(
        height=1,
        width=n
    )

    pc2.header.frame_id = "point_frame"
    pc2.header.stamp = rospy.Time.now()

    # fields - as described by the documentation
    fields = ['x', 'y', 'z']
    size_float32 = np.dtype(np.float32).itemsize

    # generator for pointfield with:
    #   name = names in fields array
    #   offset = field number * size of data in a field
    #   datatype = float32 --> if different fields have different 
    #       datatypes then the iterator will not work, perhaps 
    #       define the fields in a dict
    #   count = 1 ( 1 object per field)
    pc2.fields = [PointField(name=name, offset=i*size_float32, datatype=PointField.FLOAT32, count=1) for i, name in enumerate(fields)]

    # x and y points do not change
    x = np.array([np.linspace(0,1,50) for _ in range(50)])
    y = np.transpose(x)

    x = np.ndarray.flatten(x)
    y = np.ndarray.flatten(y)

    z = np.sin(x * 2*np.pi*0.5 + time.time())*0.5 + np.sin(y * 2*np.pi*0.25 + time.time())*0.25 + 0.5

    points = []
    for i in range(len(x)):
        points.append(x[i])
        points.append(y[i])
        points.append(z[i])
    points = np.array(points)
    # represents the points in a binary blob as float32's
    # float32's take up 32 bits = 4 bytes
    pc2.data = points.astype(np.float32).tobytes()

    # one point has 3 float32, therefore 12 bytes req.
    pc2.point_step = len(pc2.fields) * size_float32
    
    # num data points (1000) * num fields per point (3)
    pc2.row_step = pc2.point_step * n 
    return pc2


def cloudgen():
    # publish a sensor_msgs.PointCloud2 msg to topic pt_cloud_gen @ 1Hz 
    pub = rospy.Publisher("input", PointCloud2, queue_size=1)
    rospy.init_node("pt_cloud_gen", anonymous=True)
    rate = rospy.Rate(24)

    while not rospy.is_shutdown():

        pub.publish(magic_carpet(50*50))

        rate.sleep()
        


if __name__ == '__main__':
    try:
        cloudgen()
    except rospy.ROSInterruptException:
        pass