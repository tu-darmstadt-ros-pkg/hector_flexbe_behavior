#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_argos_states.get_path_state import GetPathState
from flexbe_argos_states.follow_waypoints_state import FollowWaypointsState
from flexbe_states.wait_state import WaitState
from flexbe_argos_states.reverse_list_state import ReverseListState
from flexbe_argos_states.blink_headlight_state import BlinkHeadlightState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 28 2017
@author: Gabriel Huettenberger
'''
class GPSfollowwaypointsSM(Behavior):
	'''
	Follow waypoints for outdoor challenge
	'''


	def __init__(self):
		super(GPSfollowwaypointsSM, self).__init__()
		self.name = 'GPS follow waypoints'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:95
			OperatableStateMachine.add('get_waypoints',
										GetPathState(pathTopic='/path_to_follow'),
										transitions={'succeeded': 'follow_waypoints', 'failed': 'follow_waypoints'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoints2': 'waypoints2'})

			# x:254 y:81
			OperatableStateMachine.add('follow_waypoints',
										FollowWaypointsState(desired_speed=0.8, position_tolerance=0, angle_tolerance=3, rotate_to_goal=0, reexplore_time=5),
										transitions={'reached': 'headlight_on', 'failed': 'follow_waypoints', 'reverse': 'reverse_waypoints'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'reverse': Autonomy.Full},
										remapping={'waypoints': 'waypoints', 'waypoints2': 'waypoints2'})

			# x:745 y:183
			OperatableStateMachine.add('wait_2',
										WaitState(wait_time=2),
										transitions={'done': 'headlight_off'},
										autonomy={'done': Autonomy.Off})

			# x:262 y:284
			OperatableStateMachine.add('reverse_waypoints',
										ReverseListState(),
										transitions={'succeeded': 'follow_waypoints', 'failed': 'follow_waypoints'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoints2': 'waypoints2'})

			# x:501 y:184
			OperatableStateMachine.add('headlight_on',
										BlinkHeadlightState(command=True),
										transitions={'succeeded': 'wait_2', 'failed': 'wait_2'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})

			# x:710 y:27
			OperatableStateMachine.add('headlight_off',
										BlinkHeadlightState(command=False),
										transitions={'succeeded': 'follow_waypoints', 'failed': 'follow_waypoints'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
