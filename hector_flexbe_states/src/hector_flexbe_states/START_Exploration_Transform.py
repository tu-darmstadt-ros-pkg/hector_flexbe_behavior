#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher


from smach import CBState
from rospy import Time
from actionlib_msgs.msg import GoalStatus
from hector_move_base_msgs.msg import MoveBaseActionResult
from hector_move_base_msgs.msg import MoveBaseActionExplore




class START_Exploration_Transform(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(START_Exploration_Transform, self).__init__(outcomes = ['succeeded'], input_keys=['goalId'], output_keys = ['goalId'])
		
		self._topic_explore = 'move_base/explore'
		self._pub = ProxyPublisher({self._topic_explore: MoveBaseActionExplore})



	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		return 'succeeded' # One of the outcomes declared above.
		

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.
		
		# userdata.goalId = 'none'

		actionGoal = MoveBaseActionExplore()
        	actionGoal.header.stamp = Time.now()
        
        	actionGoal.goal_id.id = 'explore_' + str(actionGoal.header.stamp.secs) + '.' + str(actionGoal.header.stamp.nsecs)
		
		# added for test
		# Logger.loginfo(actionGoal.goal_id.id)
        	
		actionGoal.goal_id.stamp = actionGoal.header.stamp
        
        	userdata.goalId = actionGoal.goal_id.id
		self._pub.publish(self._topic_explore, actionGoal)
	
        	return 'succeeded'
		


	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started
		pass

	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
		
