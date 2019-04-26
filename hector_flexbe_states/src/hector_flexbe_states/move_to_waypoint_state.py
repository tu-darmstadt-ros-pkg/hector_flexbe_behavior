#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from move_base_lite_msgs.msg import MoveBaseAction, MoveBaseGoal, ErrorCodes, PlanPathOptions

from geometry_msgs.msg import PoseStamped

'''
Created on 26.04.2019

@author: Gabriel Huettenberger
'''

class MoveToWaypointState(EventState):
	'''
	Lets the robot move to a given waypoint.

	-- desired_speed          float64              Driving speed.

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

	def __init__(self, desired_speed=0, position_tolerance=0, angle_tolerance=0, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=True):
		'''
		Constructor
		'''
		super(MoveToWaypointState, self).__init__(outcomes=['reached', 'failed', 'stuck'],
											input_keys=['waypoint'])
		
		self._action_topic = '/move_base'
		self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
		self._action_goal = MoveBaseGoal()
		self._reexplore_time = reexplore_time
		self._failed = False
		self._reached = False
		self._reverse = False
		self._desired_speed = desired_speed
		self._position_tolerance = position_tolerance
		self._angle_tolerance = angle_tolerance
		self._rotate_to_goal = rotate_to_goal
		self._use_planning = use_planning
		self._reverse_allowed = reverse_allowed
		self._reverse_forced = reverse_forced

		
		
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
		

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			if result.result.val == ErrorCodes.SUCCESS:
				self._reached = True
				return 'reached'
			if result.result.val == ErrorCodes.STUCK_DETECTED:
				return 'stuck'			
			else:
				self._failed = True
				Logger.logwarn(result.result.val)
				return 'failed'
		temp_time = rospy.get_rostime() - self._start_time
 		if (temp_time.to_sec() > self._reexplore_time):
			self._client.send_goal(self._action_topic, self._action_goal)
			self._start_time = rospy.get_rostime()

			
	def on_enter(self, userdata):
		self._failed = False
		self._reached = False

		self._start_time = rospy.get_rostime()
		self._action_goal.target_pose = userdata.waypoint
		self._action_goal.follow_path_options.desired_speed = self._desired_speed
		self._action_goal.follow_path_options.goal_pose_position_tolerance = self._position_tolerance
		self._action_goal.follow_path_options.goal_pose_angle_tolerance = self._angle_tolerance
		self._action_goal.follow_path_options.reverse_allowed = self._reverse_allowed
		self._action_goal.follow_path_options.rotate_front_to_goal_pose_orientation = self._rotate_to_goal
		if self._use_planning == False:
                        self._action_goal.plan_path_options.planning_approach = 1
		if self._action_goal.target_pose.header.frame_id == "":
			self._action_goal.target_pose.header.frame_id = "world"

		try:
			self._client.send_goal(self._action_topic, self._action_goal)
		except Exception as e:
#			Logger.logwarn('Failed to send motion request to waypoint (%(x).3f, %(y).3f):\n%(err)s' % {
#				'err': str(e),
#				'x': userdata.waypoint.pose.position.x,
#				'y': userdata.waypoint.pose.position.y
#			})
			self._failed = True
		
		Logger.loginfo('Driving to next waypoint')
			

	def on_stop(self):
		try:
			if self._client.is_available(self._action_topic) \
			and not self._client.has_result(self._action_topic):
				self._client.cancel(self._action_topic)
		except:
			# client already closed
			pass

	def on_pause(self):
		self._client.cancel(self._action_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
