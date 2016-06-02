#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher


from smach import CBState
from rospy import Time
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
import math



class Drive_to_new(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(Drive_to_new, self).__init__(outcomes = ['succeeded'], input_keys = ['pose'])



        	self._topic = '/move_base/simple_goal'
		self._pub = ProxyPublisher({self._topic: PoseStamped})
        	

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		##if rospy.Time.now() - self._start_time < self._target_time:
		return 'succeeded' # One of the outcomes declared above.
		

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# The following code is just for illustrating how the behavior logger works.
		# Text logged by the behavior logger is sent to the operator and displayed in the GUI.

		##time_to_wait = rospy.Time.now() - self._start_time - self._target_time

		##if time_to_wait > 0:
			##Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)

		target_pose = PoseStamped()
		target_pose.pose = userdata.pose
		target_pose.header.stamp = Time.now()
		target_pose.header.frame_id = 'map'
		self._pub.publish(self._topic, target_pose)

		#goal = MoveBaseActionGoal()
       		#goal.header.stamp = Time.now()
        	#goal.header.frame_id = userdata.frameId
        
        	#goal.goal.target_pose.pose.position = userdata.pose.position
        	#goal.goal.distance = userdata.params['distance']
        
        	# "straighten up" given orientation
        	#yaw = euler_from_quaternion([userdata.pose.orientation.x, userdata.pose.orientation.y, userdata.pose.orientation.z, 		self.pose.orientation.w])[2]
        	#goal.goal.target_pose.pose.orientation.x = 0
        	#goal.goal.target_pose.pose.orientation.y = 0
        	#goal.goal.target_pose.pose.orientation.z = math.sin(yaw/2)
        	#goal.goal.target_pose.pose.orientation.w = math.cos(yaw/2)
        	#goal.goal.target_pose.header.frame_id = userdata.frameId
        
        	#goal.goal_id.id = 'driveTo' + str(goal.header.stamp.secs) + '.' + str(goal.header.stamp.nsecs)
        	#goal.goal_id.stamp = goal.header.stamp

        	#userdata.goalId = goal.goal_id.id
	      	
		#self._pub.publish(self._topic_move_base_goal, goal)
        
        	return 'succeeded'
		


	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		# In this example, we use this event to set the correct start time.
		##self._start_time = rospy.Time.now()

		pass

	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
		
