#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyPublisher

from move_base_lite_msgs.msg import MoveBaseAction, MoveBaseGoal, ErrorCodes, PlanPathOptions

from geometry_msgs.msg import PoseStamped, Twist

import tf2_ros
import tf2_geometry_msgs

'''
Created on 26.04.2019

@author: Gabriel Huettenberger
'''

class MoveToRecoveryState(EventState):
	'''
	Lets the robot move to a given waypoint.


	-- position_tolerance     float64              Goal position tolerance.

	-- angle_tolerance        float64              Goal rotation tolerance.

	-- rotate_to_goal         bool                 Align to goal.

	-- reexplore_time         int		       Frequency at which the path is replanned.

	-- reverse_allowed	  bool		       If the robot is allowed to drive backwards.

	-- use_planning		  bool		       If the ignores obstacles or uses planning.

	># waypoints		PoseStamped[]		Specifies the waypoints to which the robot should move.

	<= reached 						Robot is now located at the last waypoint.
	<= failed 						Failed to send a motion request to the action server.
	<= stuck 						Robot is stuck

	'''

	def __init__(self, position_tolerance=0, angle_tolerance=0, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=True):
		'''
		Constructor
		'''
		super(MoveToRecoveryState, self).__init__(outcomes=['reached', 'failed', 'stuck'],
											input_keys=['waypoint', 'speed'])
		
		self._action_topic = '/move_base'
		self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
		self._cmd_vel_publisher = ProxyPublisher({'/cmd_vel': Twist})
		self._action_goal = MoveBaseGoal()
		self._reexplore_time = reexplore_time
		self._failed = False
		self._reached = False
		self._reverse = False
		self._stuck = False
		self._position_tolerance = position_tolerance
		self._angle_tolerance = angle_tolerance
		self._rotate_to_goal = rotate_to_goal
		self._use_planning = use_planning
		self._reverse_allowed = reverse_allowed
		self._reverse_forced = reverse_forced
		self._tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._reverse:
			return 'reverse'
		if self._failed:
			return 'failed'
		if self._reached:
			return 'reached'
		if self._stuck:
			return 'stuck'
		

		self._cmd_vel_publisher.publish('/cmd_vel', self._cmd_vel)
			
		temp_time = rospy.get_rostime() - self._start_time
 		if (temp_time.to_sec() > self._reexplore_time):

			self._start_time = rospy.get_rostime()
			return 'reached'

			
	def on_enter(self, userdata):
		self._failed = False
		self._reached = False
		self._reverse = False
		self._stuck = False
		

		transform = self._tf_buffer.lookup_transform('base_link', 'world',rospy.get_rostime(),rospy.Duration(1.0)) 

		pose_transformed = tf2_geometry_msgs.do_transform_pose(userdata.waypoint, transform)
		self._cmd_vel = Twist()
		if(pose_transformed.pose.position.x > 0):
			self._cmd_vel.linear.x = 0.3
		else:
			self._cmd_vel.linear.x = -0.3
		self._cmd_vel_publisher.publish('/cmd_vel', self._cmd_vel)
		self._start_time = rospy.get_rostime()
#		
		
		Logger.loginfo('Driving to next waypoint')
			

	def on_stop(self):
		Logger.logwarn("In on_stop of move_to_waypoint")
		try:
			if self._client.is_available(self._action_topic):
				self._client.cancel(self._action_topic)
		except:
			# client already closed
			pass
	
	def on_exit(self, userdata):
		Logger.logwarn("In on_exit of move_to_waypoint")
		try:
			if self._client.is_available(self._action_topic):
				self._client.cancel(self._action_topic)
		except:
			# client already closed
			pass

	def on_pause(self):
		self._client.cancel(self._action_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
