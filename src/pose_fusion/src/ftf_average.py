#!/usr/bin/env python3

import rospy
import numpy as np
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from geometry_msgs.msg import Vector3, Quaternion, PoseWithCovarianceStamped
from tf import transformations, TransformListener



class ftf_average():
    def __init__(self):
        self.ftf_sub = rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.transform_pose)
        self.pose_pub = rospy.Publisher(f"/pose/filtered", PoseWithCovarianceStamped, queue_size=1)
        self.listener = TransformListener()
        # self.seq = 0

    def conversion_cb(self, ftf_arr : FiducialTransformArray):
        pwcs = PoseWithCovarianceStamped()
        pwcs.header.stamp = ftf_arr.header.stamp
        pwcs.header.frame_id = "base_link"
        
        x = []
        y = []
        z = []
        roll = []
        pitch = []
        yaw = []

        ftf : FiducialTransform
        for ftf in ftf_arr.transforms:
            euler = transformations.euler_from_quaternion([ftf.transform.rotation.x, ftf.transform.rotation.y, ftf.transform.rotation.z, -ftf.transform.rotation.w,])
            roll.append(euler[0])
            pitch.append(euler[1])
            yaw.append(euler[2])

            x.append(-ftf.transform.translation.x)
            y.append(-ftf.transform.translation.y)
            z.append(-ftf.transform.translation.z)
        
        cov = np.array([[np.var(x), 0, 0, 0, 0, 0],
                        [0, np.var(y), 0, 0, 0, 0],
                        [0, 0, np.var(z), 0, 0, 0],
                        [0, 0, 0, np.var(roll), 0, 0],
                        [0, 0, 0, 0, np.var(pitch), 0],
                        [0, 0, 0, 0, 0, np.var(yaw)]])

        q = transformations.quaternion_from_euler(np.mean(roll), np.mean(pitch), np.mean(yaw))
        pwcs.pose.pose.position = Vector3(np.mean(x), np.mean(y), np.mean(z))
        pwcs.pose.pose.orientation = Quaternion(*q)
        # pwcs.pose.covariance = cov.flatten()

        # publish
        self.pose_pub.publish(pwcs)
        # self.seq += 1
    
    def transform_pose(self, ftf_arr : FiducialTransformArray):
        pwcs = PoseWithCovarianceStamped()
        pwcs.header.stamp = ftf_arr.header.stamp
        pwcs.header.frame_id = f"base_link"
        
        x = []
        y = []
        z = []
        roll = []
        pitch = []
        yaw = []

        ftf : FiducialTransform
        for ftf in ftf_arr.transforms:
            # apply a transform back to the base link based on the fiducial id and known aruco positions
            self.listener.waitForTransform(f"aruco_{ftf.fiducial_id}_known", f"base_link", rospy.Time.now(), rospy.Duration(0.1))
            known_tf = self.listener.lookupTransform(f"aruco_{ftf.fiducial_id}_known", f"base_link", rospy.Time.now())

            # apply the transform to the fiducial transform
            # ftf.transform = ftf.transform * known_tf
            
            euler = transformations.euler_from_quaternion([ftf.transform.rotation.x, ftf.transform.rotation.y, ftf.transform.rotation.z, -ftf.transform.rotation.w,])
            roll.append(euler[0])
            pitch.append(euler[1])
            yaw.append(euler[2])

            x.append(-ftf.transform.translation.x)
            y.append(-ftf.transform.translation.y)
            z.append(-ftf.transform.translation.z)
        
        cov = np.array([[np.var(x), 0, 0, 0, 0, 0],
                        [0, np.var(y), 0, 0, 0, 0],
                        [0, 0, np.var(z), 0, 0, 0],
                        [0, 0, 0, np.var(roll), 0, 0],
                        [0, 0, 0, 0, np.var(pitch), 0],
                        [0, 0, 0, 0, 0, np.var(yaw)]])

        q = transformations.quaternion_from_euler(np.mean(roll), np.mean(pitch), np.mean(yaw))
        pwcs.pose.pose.position = Vector3(np.mean(x), np.mean(y), np.mean(z))
        pwcs.pose.pose.orientation = Quaternion(*q)
        # pwcs.pose.covariance = cov.flatten()

        # publish
        self.pose_pub.publish(pwcs)
        # self.seq += 1



if __name__ == "__main__":
    rospy.init_node("converter")
    n = ftf_average()
    while not rospy.is_shutdown():
        rospy.spin()