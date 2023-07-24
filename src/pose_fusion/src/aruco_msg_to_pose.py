#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import transformations



class converter():
    def __init__(self):
        self.ftf_sub = rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.conversion_cb)
        self.pose_pub = rospy.Publisher(f"pose/1", PoseWithCovarianceStamped, queue_size=1)
        self.seq = 0
        self.cov = [
            0.005, 0, 0, 0, 0, 0,
            0, 0.005, 0, 0, 0, 0,
            0, 0, 0.005, 0, 0, 0,
            0, 0, 0, 0.001, 0, 0,
            0, 0, 0, 0, 0.001, 0,
            0, 0, 0, 0, 0, 0.001]

    def conversion_cb(self, ftf_arr : FiducialTransformArray):
        # type annotation, ftf must be a FiducialTransform
        ftf : FiducialTransform
        for ftf in ftf_arr.transforms:
            # I hate line below but its fine for now
            self.pose_pub.__init__(f"pose/{ftf.fiducial_id}", PoseWithCovarianceStamped, queue_size=1)
            # convert message
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = ftf_arr.header.stamp
            pose.header.frame_id = "base_link"
            pose.header.seq = self.seq
            # inverse tf
            q = ftf.transform.rotation
            q.w = -q.w
            t = ftf.transform.translation
            t.x = -t.x
            t.y = -t.y
            t.z = -t.z
            # store in new msg
            pose.pose.pose.position = t
            pose.pose.pose.orientation = q
            pose.pose.covariance = self.cov
            # publish
            self.pose_pub.publish(pose)
            # update seq
            self.seq += 1



if __name__ == "__main__":
    rospy.init_node("converter")
    n = converter()
    while not rospy.is_shutdown():
        rospy.spin()