#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_argos_states.get_path_state import GetPathState
from hector_flexbe_states.get_waypoint_from_array_state import GetWaypointFromArrayState
from hector_flexbe_states.get_recovery_info_state import GetRecoveryInfoState
from hector_flexbe_states.move_to_waypoint_state import MoveToWaypointState as hector_flexbe_states__MoveToWaypointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 11 2019
@author: Gabriel Huettenberger
'''
class WaypointmissionSM(Behavior):
	'''
	Follow a path of waypoints
	'''


	def __init__(self):
		super(WaypointmissionSM, self).__init__()
		self.name = 'Waypoint mission'

		# parameters of this behavior
		self.add_parameter('speed', 0.2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:226 y:244, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.speed = self.speed
		_state_machine.userdata.current_waypoint = PointStamped()
		_state_machine.userdata.remaining_waypoints = PointStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:89 y:61
			OperatableStateMachine.add('get_path',
										GetPathState(pathTopic='/path_to_follow'),
										transitions={'succeeded': 'get_current_waypoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints2': 'remaining_waypoints'})

			# x:301 y:66
			OperatableStateMachine.add('get_current_waypoint',
										GetWaypointFromArrayState(position=0),
										transitions={'succeeded': 'move_to_next_waypoint', 'empty': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'waypoints': 'remaining_waypoints', 'waypoint': 'current_waypoint'})

			# x:582 y:281
			OperatableStateMachine.add('get_recovery_point',
										GetRecoveryInfoState(service_topic='/trajectory_recovery_info'),
										transitions={'success': 'move_to_recovery_point', 'failed': 'move_to_next_waypoint'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'recovery_point'})

			# x:573 y:74
			OperatableStateMachine.add('move_to_next_waypoint',
										hector_flexbe_states__MoveToWaypointState(desired_speed=0, position_tolerance=0, angle_tolerance=0, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=True),
										transitions={'reached': 'get_current_waypoint', 'failed': 'failed', 'stuck': 'get_recovery_point'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'waypoint': 'current_waypoint'})

			# x:794 y:165
			OperatableStateMachine.add('move_to_recovery_point',
										hector_flexbe_states__MoveToWaypointState(desired_speed=0, position_tolerance=0, angle_tolerance=0, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=True),
										transitions={'reached': 'move_to_next_waypoint', 'failed': 'move_to_next_waypoint', 'stuck': 'get_recovery_point'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'waypoint': 'recovery_point'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
