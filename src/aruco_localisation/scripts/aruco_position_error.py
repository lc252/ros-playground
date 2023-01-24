#!/usr/bin/env python3

# a script to quantify and plot the accuracy and variation in aruco pose estimation

import rospy
import matplotlib.pyplot as plt
import pandas as pd
import tf
import time


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    data = []
    df = pd.DataFrame()

    while not rospy.is_shutdown() and len(data) < 1000:
        
        # (xyz, xyzw)
        listener.waitForTransform("fiducial_226", "camera_color_optical_frame", rospy.Time(0), rospy.Duration(1))
        (trans,rot) = listener.lookupTransform('fiducial_226', 'camera_color_optical_frame', rospy.Time(0))
        data.append([*trans, *rot])
        
        df = pd.DataFrame(data)

        print(df.tail())
        rate.sleep()
    
    print(df.describe(include="all"))
    plt.plot(df[0])