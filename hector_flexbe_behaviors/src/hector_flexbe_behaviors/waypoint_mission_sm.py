#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.get_recovery_info_state import GetRecoveryInfoState
from hector_flexbe_states.move_to_waypoint_state import MoveToWaypointState as hector_flexbe_states__MoveToWaypointState
from hector_flexbe_states.write_3d_map_state import Write3dMapState
from hector_flexbe_states.write_2d_map_state import Write2dMapState
from hector_flexbe_states.get_waypoint_from_array_state import GetWaypointFromArrayState
from hector_flexbe_states.switch_mapping import SwitchMapping
from hector_flexbe_states.add_waypoint_to_array_state import AddWaypointToArrayState
from hector_flexbe_states.move_to_recovery_state import MoveToRecoveryState
from hector_flexbe_states.get_path_from_service_state import GetPathFromServiceState as hector_flexbe_states__GetPathFromServiceState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PointStamped
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
		self.add_parameter('usePlanning', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:233 y:309, x:188 y:441
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['speed', 'reexplore_time', 'pose'])
		_state_machine.userdata.speed = 0.2
		_state_machine.userdata.reexplore_time = 5
		_state_machine.userdata.counter = 0
		_state_machine.userdata.pose = PointStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:31 y:39
			OperatableStateMachine.add('get_path1',
										hector_flexbe_states__GetPathFromServiceState(service_topic='/path_to_follow'),
										transitions={'succeeded': 'switch_mapping_on', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints2': 'remaining_waypoints'})

			# x:580 y:152
			OperatableStateMachine.add('move_to_next_waypoint',
										hector_flexbe_states__MoveToWaypointState(position_tolerance=0.1, angle_tolerance=3, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=self.usePlanning),
										transitions={'reached': 'write_3d_map', 'failed': 'move_to_next_waypoint', 'stuck': 'get_recovery_point'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'waypoint': 'current_waypoint', 'speed': 'speed'})

			# x:903 y:153
			OperatableStateMachine.add('move_to_recovery_point',
										hector_flexbe_states__MoveToWaypointState(position_tolerance=0.1, angle_tolerance=3, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=True),
										transitions={'reached': 'move_to_next_waypoint', 'failed': 'move_to_next_waypoint', 'stuck': 'second_recovery'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'waypoint': 'recovery_point', 'speed': 'speed'})

			# x:589 y:21
			OperatableStateMachine.add('write_3d_map',
										Write3dMapState(service_topic='/worldmodel_main/save_map', save_path='/octomaps/waypoints'),
										transitions={'success': 'write_2d_map', 'failed': 'write_2d_map'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:322 y:20
			OperatableStateMachine.add('write_2d_map',
										Write2dMapState(writer_topic='/syscommand'),
										transitions={'success': 'switch_mapping_off', 'failed': 'switch_mapping_off'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:307 y:149
			OperatableStateMachine.add('get_current_waypoint',
										GetWaypointFromArrayState(position=0),
										transitions={'succeeded': 'move_to_next_waypoint', 'empty': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'waypoints': 'remaining_waypoints', 'waypoint': 'current_waypoint'})

			# x:436 y:82
			OperatableStateMachine.add('switch_mapping_off',
										SwitchMapping(enable=False),
										transitions={'succeeded': 'get_current_waypoint'},
										autonomy={'succeeded': Autonomy.Off})

			# x:196 y:75
			OperatableStateMachine.add('switch_mapping_on',
										SwitchMapping(enable=True),
										transitions={'succeeded': 'add_waypoint'},
										autonomy={'succeeded': Autonomy.Off})

			# x:111 y:222
			OperatableStateMachine.add('add_waypoint',
										AddWaypointToArrayState(),
										transitions={'succeeded': 'get_current_waypoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pose', 'waypoints': 'remaining_waypoints'})

			# x:864 y:19
			OperatableStateMachine.add('second_recovery',
										MoveToRecoveryState(position_tolerance=0, angle_tolerance=0, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=True),
										transitions={'reached': 'move_to_next_waypoint', 'failed': 'move_to_next_waypoint', 'stuck': 'move_to_next_waypoint'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'waypoint': 'recovery_point', 'speed': 'speed'})

			# x:589 y:320
			OperatableStateMachine.add('get_recovery_point',
										GetRecoveryInfoState(service_topic='/trajectory_recovery_info'),
										transitions={'success': 'move_to_recovery_point', 'failed': 'move_to_next_waypoint'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'recovery_point'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
