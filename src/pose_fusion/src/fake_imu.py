#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Transform, Vector3, Quaternion
import numpy as np



class fake_imu():
    def __init__(self):
        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1/60), self.publish_imu)

    def publish_imu(self, msg):
        # create 4 fiducial transforms with some noise and publish as array
        ori = Quaternion(0,0,0,1)
        ori_cov = np.random.rand(9) * 0.001
        ang = np.random.rand(3) * 0.001
        ang_cov = np.random.rand(9) * 0.001
        lin = np.array([0,0,9.81]) + np.random.rand(3) * 0.001
        lin_cov = np.random.rand(9) * 0.001
        header = Header(0, rospy.Time.now(), "imu_frame")
        imu = Imu(header, ori, ori_cov, Vector3(*ang), ang_cov, Vector3(*lin), lin_cov)
        self.imu_pub.publish(imu)



if __name__ == "__main__":
    rospy.init_node("fake_aruco_publisher")
    n = fake_imu()
    while not rospy.is_shutdown():
        rospy.spin()
