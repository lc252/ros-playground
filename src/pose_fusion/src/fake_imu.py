#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Transform, Vector3, Quaternion
import numpy as np



class fake_imu():
    def __init__(self):
        self.imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=1)
        self.mag_pub = rospy.Publisher("imu/mag", MagneticField, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1/60), self.publish_imu)

    def publish_imu(self, msg):
        # create 4 fiducial transforms with some noise and publish as array
        ori = Quaternion(-1,0,0,0)      # most imus do not provide an orientation estimate. set -1,0,0,0 to indicate this
        ori_cov = np.zeros(9) # np.random.rand(9) * 0.001
        ang = np.random.rand(3) * 0.001
        ang_cov = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        lin = np.array([0,0,9.81]) + np.random.rand(3) * 0.001
        lin_cov = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        header = Header(0, rospy.Time(0), "imu_frame")
        imu = Imu(header, ori, ori_cov, Vector3(*ang), ang_cov, Vector3(*lin), lin_cov)
        self.publish_mag()
        self.imu_pub.publish(imu)

    def publish_mag(self):
        # create fake mag data
        mag = np.array([485, 455, -204]) + (np.random.rand(3) - 0.5)
        mag_cov = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        header = Header(0, rospy.Time(0), "imu_frame")
        mag = MagneticField(header, Vector3(*mag), mag_cov)
        self.mag_pub.publish(mag)



if __name__ == "__main__":
    rospy.init_node("fake_aruco_publisher")
    n = fake_imu()
    while not rospy.is_shutdown():
        rospy.spin()
