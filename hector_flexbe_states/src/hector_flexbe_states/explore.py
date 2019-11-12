#!/usr/bin/env python
import rospy

from flexbe_core.proxy import ProxyActionClient, ProxyPublisher

from flexbe_core import EventState, Logger
from rospy import Time
from move_base_lite_msgs.msg import ExploreAction, ExploreGoal
from dynamic_reconfigure.client import Client

from std_msgs.msg import Float64




class Explore(EventState):
	'''
	Starts the Exploration Task via /move_base_lite

	># speed		float64		Speed of the robot
	># reexplore_time	int		Duration after which to trigger reexploration

	<= succeeded				Exploration Task was successful
	<= failed 				Exploration Task failed
	<= stuck				Robot is stuck

	'''

	def __init__(self):
		super(Explore, self).__init__(outcomes = ['succeeded', 'failed', 'stuck'], input_keys =['speed', 'reexplore_time', 'first_call'], output_keys = ['first_call'])
		
		self._action_topic = '/explore'
		self._move_client = ProxyActionClient({self._action_topic: ExploreAction})
		self._start_time = rospy.Time.now()
		self._succeeded = False
		self._failed = False
		self._action_goal = ExploreGoal()
		self._topic = '/sweep_velocity_command'
		self._pub = ProxyPublisher({self._topic: Float64})


	def execute(self, userdata):

		if self._move_client.has_result(self._action_topic):
			result = self._move_client.get_result(self._action_topic)
			if result.result.val == 1:
				self._move_client.send_goal(self._action_topic, self._action_goal)
                        if result.result.val == -8:
                                return 'stuck'
			else:
				self._failed = True
				Logger.logwarn('Exploration failed!')
				return 'failed'
		temp_time = rospy.get_rostime() - self._start_time;
 		if (temp_time.to_sec() > userdata.reexplore_time) and userdata.reexplore_time > 0:
			self._start_time = rospy.get_rostime()
			self._action_goal.reset_stuck_history = False
			self._move_client.send_goal(self._action_topic, self._action_goal)

		

	def on_enter(self, userdata):
			
			
		self._succeeded = False
		self._failed = False
		self._start_time = rospy.get_rostime()
		self._action_goal.desired_speed = userdata.speed
		if userdata.first_call == True:
			self._action_goal.reset_stuck_history = True
			userdata.first_call = False
			Logger.loginfo('first call true')
		else:
			self._action_goal.reset_stuck_history = False
			Logger.loginfo('first call false')
		

		try:
			if self._move_client.is_active(self._action_topic):
				self._move_client.cancel(self._action_topic)
			self._move_client.send_goal(self._action_topic, self._action_goal)
		except Exception as e:
			self._failed = True
		
		


	def on_exit(self, userdata):
		self._move_client.cancel(self._action_topic)
		self._pub.publish(self._topic, Float64())


	def on_start(self):
		pass

	def on_stop(self):
		pass
		
