#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from geometry_msgs.msg import Transform, Vector3, Quaternion
import numpy as np



class fake_aruco():
    def __init__(self):
        self.ftf_pub = rospy.Publisher("fiducial_transforms", FiducialTransformArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_ftf)

    def publish_ftf(self, msg):
        # create 4 fiducial transforms with some noise and publish as array
        ftf_array = FiducialTransformArray()
        ftf_array.header.frame_id = "camera_color_optical_frame"
        ftf_array.header.stamp = rospy.Time.now()
        for id in range(1,5):
            rand_v3 = np.random.rand(3) * 0.1
            rand_q = np.random.rand(4) * 0.1
            tf = Transform(Vector3(*rand_v3), Quaternion(*rand_q))
            ftf = FiducialTransform(id, tf, 0, 0, 0)
            ftf_array.transforms.append(ftf)
        self.ftf_pub.publish(ftf_array)



if __name__ == "__main__":
    rospy.init_node("fake_aruco_publisher")
    n = fake_aruco()
    while not rospy.is_shutdown():
        rospy.spin()