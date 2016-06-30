#!/usr/bin/env python
import math
import rospy
import tf

from flexbe_core import EventState, Logger

from nav_msgs.msg import Path

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class SparsePath(EventState):
	'''
	Removes points which are too dense

	-- max_dist 		float 		If a point is further away than this value, it will be added.
	-- max_angle 		float 		If orientation difference is larger than this value, the point will be added.
	-- min_dist 		float 		If a point is closer than this value, it will be discarded no matter the angle.

	># path				Path		Array of Positions of the robot.
	#> path				Path		Array of Positions of the robot.

	<= done 						Updated path.
	'''

	def __init__(self, max_dist, max_angle, min_dist = 0.0):
		'''
		Constructor
		'''
		super(SparsePath, self).__init__(outcomes=['done'], input_keys=['path'], output_keys=['path'])
		
		self._max_dist = max_dist
		self._max_angle = max_angle
		self._min_dist = min_dist
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		return 'done'
		

			
	def on_enter(self, userdata):

		path = Path()
		path.header = userdata.path.header
		
		for p in userdata.path.poses:
			# always add first pose
			if len(path.poses) == 0:
				path.poses.append(p)
				path.header.stamp = rospy.Time.now() + rospy.Duration(.5)
				continue

			last_pose = path.poses[-1]
			p.header.stamp = last_pose.header.stamp + rospy.Duration(1)

			# check distance
			l_xyz = [last_pose.pose.position.x, last_pose.pose.position.y, last_pose.pose.position.z]
			xyz = [p.pose.position.x, p.pose.position.y, p.pose.position.z]
			dist = math.sqrt(sum((p-l)*(p-l) for l,p in zip(l_xyz, xyz)))
			if dist >= self._max_dist:
				path.poses.append(p)
				continue
			if dist < self._min_dist:
				continue

			# check orientation diff
			l_q = last_pose.pose.orientation
			l_rpy = tf.transformations.euler_from_quaternion([l_q.x, l_q.y, l_q.z, l_q.w])
			q = p.pose.orientation
			rpy = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
			if abs(rpy[2] - l_rpy[2]) >= self._max_angle:
				path.poses.append(p)
				continue

		userdata.path = path
	

