#!/usr/bin/env python

import rospy
import numpy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from fiducial_msgs.msg import FiducialTransformArray
from sensor_msgs.msg import Imu

class vel_manipulator:

	t0 = Vector3(0,0,0)
	r0 = Quaternion(0,0,0,1)
	cam_moved = False

	def __init__(self):
		# topic to see if camera moves from camera_acceleration
		sub2_topic_name ="/camera/accel/sample"
		self.cam_subscriber = rospy.Subscriber(sub2_topic_name, Imu, self.cam_callback)
		# topic to get marker transforms, from aruco_detect package
		sub_topic_name ="/fiducial_transforms"
		self.number_subscriber = rospy.Subscriber(sub_topic_name, FiducialTransformArray, self.transform_callback)


	# Functions to reverse transformations
	def reverse_translate(self, translation):

		result = Vector3()
		result.x = -translation.x
		result.y = -translation.y
		result.z = -translation.z

		return result

	def reverse_rotate(self, quaternion):
		result = Quaternion()
		result.x = quaternion.x
		result.y = quaternion.y
		result.z = quaternion.z
		result.w = -quaternion.w

		return result

	def tf_broadcaster(self, broadcast, link, parent_frame, child_frame, translation, rotation):
		
		# Function to broacast tf's
		link.header.stamp = rospy.Time.now()
		link.header.frame_id = parent_frame
		link.child_frame_id = child_frame

		link.transform.translation = translation

		link.transform.rotation = rotation
		broadcast.sendTransform(link)

		return

	def cam_callback(self, msg):

		# reads the magnitude of the linear acceleration vector of the camera and decides whether it is moving
		move_c =[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
		# this is roughly the value of it when stationary 
		base_value = 9.677

		move = numpy.linalg.norm(move_c)
		if move > base_value + 0.6:
			self.cam_moved = True
		elif move < base_value - 0.6:
			self.cam_moved = True
		else:
			self.cam_moved = False

	def transform_callback(self, msg):

		# Read fiducial marker transforms
		marker_transforms = msg.transforms
		# dynamic transform broadcaster
		br = tf2_ros.TransformBroadcaster()
		# static transform broadcaster
		tfBuffer = tf2_ros.Buffer()

		# Get all 4 aruco markers in a list
		coords = [geometry_msgs.msg.TransformStamped() for marker in range(len(marker_transforms))]

		# Transformation from camera_color_optical_frame to camera_link
		constant_quat = Quaternion(0.497,-0.504,0.496,0.503)
		constant_trans = Vector3(0.015,0.0,0.0)

		# Variables to get minimum error marker
		if len(marker_transforms) > 0:
			imerr = marker_transforms[0].image_error
			index = 0
			# This finds the minimum error aruco marker and uses that for the camera_link transformation
			for marker in range(len(marker_transforms)):		
				if marker_transforms[marker].image_error < imerr:
					imerr = marker_transforms[marker].image_error
					index = marker

			rev_translate = self.reverse_translate(marker_transforms[index].transform.translation)
			rev_rotate = self.reverse_rotate(marker_transforms[index].transform.rotation)
			# Reverse the rotation first
			self.tf_broadcaster(br, coords[index], "ARuco"+str(marker_transforms[index].fiducial_id),"temp1", self.t0, rev_rotate)
			# Reverse translation
			self.tf_broadcaster(br, coords[index], "temp1", "temp2",rev_translate, self.r0)
			# Transform into the camera link
			self.tf_broadcaster(br, coords[index], "temp2", "camera_link", constant_trans, constant_quat)

			# There should be a more elegant way to do the above with quaternion mathematics

"""		Not being used at the moment, publishing static transforms freezes the point cloud
			elif self.cam_moved == False:
			transform_marker2cam = tfBuffer.lookup_transform("camera_link", "base_scan", rospy.Time())
			self.tf_broadcaster(static_br, coords[index], "base_scan", "camera_link", transform_marker2cam.transform.translation, transform_marker2cam.transform.rotation)
"""
if __name__ == '__main__':
	node_name ="locate_camera"
	rospy.init_node(node_name)
	vel_manipulator()
	rospy.spin()