#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_argos_states.explore import Explore
from flexbe_states.wait_state import WaitState
from hector_flexbe_states.get_recovery_info_state import GetRecoveryInfoState
from flexbe_argos_states.move_to_waypoint_state import MoveToWaypointState as flexbe_argos_states__MoveToWaypointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 20 2017
@author: Gabriel Huettenberger
'''
class ExploreMissionRobocupSM(Behavior):
	'''
	Default behavior for exploration based Robocup missions
	'''


	def __init__(self):
		super(ExploreMissionRobocupSM, self).__init__()
		self.name = 'Explore Mission Robocup'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['reexplore_time', 'speed'])
		_state_machine.userdata.speed = 0.2
		_state_machine.userdata.reexplore_time = 5

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:91 y:128
			OperatableStateMachine.add('Start Exploration',
										Explore(),
										transitions={'succeeded': 'Wait', 'failed': 'Wait', 'stuck': 'get_recovery_point'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'speed': 'speed', 'reexplore_time': 'reexplore_time'})

			# x:341 y:63
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=1),
										transitions={'done': 'Start Exploration'},
										autonomy={'done': Autonomy.Off})

			# x:271 y:234
			OperatableStateMachine.add('get_recovery_point',
										GetRecoveryInfoState(service_topic='/trajectory_recovery_info'),
										transitions={'success': 'stuck_behavior', 'failed': 'Start Exploration'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})

			# x:548 y:243
			OperatableStateMachine.add('stuck_behavior',
										flexbe_argos_states__MoveToWaypointState(desired_speed=0.2, position_tolerance=0, angle_tolerance=3, rotate_to_goal=False, reverse_allowed=True),
										transitions={'reached': 'Start Exploration', 'failed': 'Start Exploration'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
