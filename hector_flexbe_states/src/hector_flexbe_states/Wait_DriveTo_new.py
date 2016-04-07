#!/usr/bin/env python
import rospy


from flexbe_core import EventState, Logger

# added
from flexbe_core.proxy import ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionResult
# from old hector behavior
# from move_base_msgs.msg import MoveBaseActionGoal
from hector_move_base_msgs.msg import MoveBaseActionExplore


class Wait_DriveTo_new(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self, useMoveBase=True):
		
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(Wait_DriveTo_new, self).__init__(outcomes=['succeeded', 'aborted', 'waiting'],input_keys=['goalId'])

		# ???
		self._abortStatus = [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]

		self._resultTopic = '/move_base/result' #if useMoveBase else 'controller/result'
		self._sub = ProxySubscriberCached({self._resultTopic: MoveBaseActionResult})
		# don't define a callback, simply request last message in execute
		

		self._succeeded = False
		self._aborted = False
		self._waiting = False

		self._result = GoalStatus.PENDING

	
        
        	

		# Store state parameter for later use.
		## self._target_time = rospy.Duration(target_time)

		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		## self._start_time = None


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		
		Logger.loginfo(' [hector_behavior] in Drive_to waiting.... ')

        	tmp = self._sub.get_last_msg(self._resultTopic)
        	self._result = GoalStatus.PENDING

        	if tmp is None:
            		Logger.loginfo(' [hector_behavior] drive to waiting because no published result')
			# ???            		
			rospy.sleep(0.5)
			self_waiting = True
            		return 'waiting'
        	if userdata.goalId == tmp.status.goal_id.id:
			self._result = tmp.status.status
        	if self._result == GoalStatus.SUCCEEDED:
			self_succeeded = True
            		return 'succeeded'
       		elif self._result in self._abortStatus:
			self_aborted = True
            		return 'aborted'
        	else:
            		Logger.loginfo(' [hector_behavior] drive to waiting because no matching goal id') 
            		rospy.sleep(0.5)
			self_waiting = True
            		return 'waiting'


	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		pass


	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		# In this example, we use this event to set the correct start time.
		## self._start_time = rospy.Time.now()
		pass


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
		
