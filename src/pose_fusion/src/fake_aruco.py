#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from geometry_msgs.msg import Transform, Vector3, Quaternion
from tf import TransformBroadcaster, transformations
import numpy as np



class fake_aruco():
    def __init__(self):
        self.ftf_pub = rospy.Publisher("fiducial_transforms", FiducialTransformArray, queue_size=1)
        self.tf_broadcaster = TransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.link_ftfs)

    def publish_ftf(self, msg):
        # create 4 fiducial transforms with some noise and publish as array
        ftf_array = FiducialTransformArray()
        ftf_array.header.frame_id = "camera_color_optical_frame"
        ftf_array.header.stamp = rospy.Time.now()
        for id in range(1,5):
            rand_v3 = np.random.normal(-1, 0.01, 3)    # normally distributed around -1
            rand_q = transformations.quaternion_from_euler(*(np.random.normal(0, 0.001, 3)), 'ryxz')
            tf = Transform(Vector3(*rand_v3), Quaternion(*rand_q))
            ftf = FiducialTransform(id, tf, 0, 0, 0)
            ftf_array.transforms.append(ftf)
            self.tf_broadcaster.sendTransform(rand_v3, rand_q, ftf_array.header.stamp, f"aruco_{id}", ftf_array.header.frame_id)
        self.ftf_pub.publish(ftf_array)
    
    def link_ftfs(self, msg):
        # create 4 fiducial transforms from different positions with some noise and publish as array
        ftf_array = FiducialTransformArray()
        ftf_array.header.frame_id = "camera_color_optical_frame"
        ftf_array.header.stamp = rospy.Time.now()

        # aruco 1
        v3 = np.array([0.25,1,-0.5]) + np.random.normal(0, 0.01, 3)    # normally distributed noise
        rand_q = transformations.quaternion_from_euler(*(np.random.normal(0, 0.001, 3)), 'ryxz')
        tf = Transform(Vector3(*v3), Quaternion(*rand_q))
        ftf = FiducialTransform(1, tf, 0, 0, 0)
        ftf_array.transforms.append(ftf)
        self.tf_broadcaster.sendTransform(v3, rand_q, ftf_array.header.stamp, f"aruco_1", ftf_array.header.frame_id)
        
        # aruco 2
        v3 = np.array([0.25,1,0]) + np.random.normal(0, 0.01, 3)    # normally distributed noise
        rand_q = transformations.quaternion_from_euler(*(np.random.normal(0, 0.001, 3)), 'ryxz')
        tf = Transform(Vector3(*v3), Quaternion(*rand_q))
        ftf = FiducialTransform(1, tf, 0, 0, 0)
        ftf_array.transforms.append(ftf)
        self.tf_broadcaster.sendTransform(v3, rand_q, ftf_array.header.stamp, f"aruco_2", ftf_array.header.frame_id)

        # aruco 3
        v3 = np.array([-0.25,1,0]) + np.random.normal(0, 0.01, 3)    # normally distributed noise
        rand_q = transformations.quaternion_from_euler(*(np.random.normal(0, 0.001, 3)), 'ryxz')
        tf = Transform(Vector3(*v3), Quaternion(*rand_q))
        ftf = FiducialTransform(1, tf, 0, 0, 0)
        ftf_array.transforms.append(ftf)
        self.tf_broadcaster.sendTransform(v3, rand_q, ftf_array.header.stamp, f"aruco_3", ftf_array.header.frame_id)

        # aruco 4
        v3 = np.array([-0.25,1,0.5]) + np.random.normal(0, 0.01, 3)    # normally distributed noise
        rand_q = transformations.quaternion_from_euler(*(np.random.normal(0, 0.001, 3)), 'ryxz')
        tf = Transform(Vector3(*v3), Quaternion(*rand_q))
        ftf = FiducialTransform(1, tf, 0, 0, 0)
        ftf_array.transforms.append(ftf)
        self.tf_broadcaster.sendTransform(v3, rand_q, ftf_array.header.stamp, f"aruco_4", ftf_array.header.frame_id)

        self.ftf_pub.publish(ftf_array)



if __name__ == "__main__":
    rospy.init_node("fake_aruco_publisher")
    n = fake_aruco()
    while not rospy.is_shutdown():
        rospy.spin()
