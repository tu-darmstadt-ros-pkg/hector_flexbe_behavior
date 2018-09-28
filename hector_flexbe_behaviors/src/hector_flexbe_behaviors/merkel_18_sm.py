#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.LookAtPattern import LookAtPattern
from flexbe_argos_states.explore import Explore
from hector_flexbe_states.detect_object import DetectObject
from hector_flexbe_states.get_recovery_info_state import GetRecoveryInfoState
from flexbe_argos_states.move_to_waypoint_state import MoveToWaypointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Sep 27 2018
@author: Gabriel Huettenberger
'''
class Merkel_18SM(Behavior):
	'''
	Behavior for Merkel visit of team Hector
	'''


	def __init__(self):
		super(Merkel_18SM, self).__init__()
		self.name = 'Merkel_18'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:258 y:364
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.speed = 0.2
		_state_machine.userdata.reexplore_time = 5
		_state_machine.userdata.pattern = 'test'
		_state_machine.userdata.pose = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365
		_sm_explore_and_detect_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'stuck'], input_keys=['speed', 'reexplore_time', 'pose'], output_keys=['victim'], conditions=[
										('finished', [('explore', 'succeeded')]),
										('failed', [('explore', 'failed')]),
										('stuck', [('explore', 'stuck')]),
										('finished', [('find_heat_source', 'found')])
										])

		with _sm_explore_and_detect_0:
			# x:30 y:40
			OperatableStateMachine.add('explore',
										Explore(),
										transitions={'succeeded': 'finished', 'failed': 'failed', 'stuck': 'stuck'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'speed': 'speed', 'reexplore_time': 'reexplore_time'})

			# x:256 y:163
			OperatableStateMachine.add('find_heat_source',
										DetectObject(),
										transitions={'found': 'finished'},
										autonomy={'found': Autonomy.Off},
										remapping={'pose': 'pose', 'victim': 'victim'})



		with _state_machine:
			# x:62 y:43
			OperatableStateMachine.add('start_head_pattern',
										LookAtPattern(),
										transitions={'succeeded': 'Explore and Detect', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pattern': 'pattern'})

			# x:451 y:178
			OperatableStateMachine.add('Explore and Detect',
										_sm_explore_and_detect_0,
										transitions={'finished': 'finished', 'failed': 'failed', 'stuck': 'get_recovery_waypoint'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'stuck': Autonomy.Inherit},
										remapping={'speed': 'speed', 'reexplore_time': 'reexplore_time', 'pose': 'pose', 'victim': 'victim'})

			# x:449 y:402
			OperatableStateMachine.add('get_recovery_waypoint',
										GetRecoveryInfoState(service_topic='/trajectory_recovery_info'),
										transitions={'success': 'stuck_behavior', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})

			# x:611 y:296
			OperatableStateMachine.add('stuck_behavior',
										MoveToWaypointState(desired_speed=self.speed, position_tolerance=0, angle_tolerance=3, rotate_to_goal=False, reverse_allowed=True),
										transitions={'reached': 'Explore and Detect', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
