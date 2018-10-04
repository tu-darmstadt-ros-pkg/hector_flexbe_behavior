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
from hector_flexbe_states.get_recovery_info_state import GetRecoveryInfoState
from hector_flexbe_states.move_to_waypoint_state import MoveToWaypointState
from hector_flexbe_states.confirm_victim import ConfirmVictim
from hector_flexbe_behaviors.rc_exp5_sm import rc_exp5SM
from hector_flexbe_states.detect_object import DetectObject
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
		self.add_behavior(rc_exp5SM, 'Explore and Detect/rc_exp5')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:339 y:380, x:408 y:329
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pattern = 'look_around'
		_state_machine.userdata.speed = 0.2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_explore_and_detect_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'stuck'], output_keys=['victim', 'pose'], conditions=[
										('finished', [('find_heat_source', 'found')]),
										('finished', [('rc_exp5', 'finished')]),
										('failed', [('rc_exp5', 'failed')])
										])

		with _sm_explore_and_detect_0:
			# x:30 y:102
			OperatableStateMachine.add('rc_exp5',
										self.use_behavior(rc_exp5SM, 'Explore and Detect/rc_exp5'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:256 y:163
			OperatableStateMachine.add('find_heat_source',
										DetectObject(),
										transitions={'found': 'finished'},
										autonomy={'found': Autonomy.Off},
										remapping={'pose': 'pose', 'victim': 'victim'})



		with _state_machine:
			# x:246 y:214
			OperatableStateMachine.add('start_head_pattern',
										LookAtPattern(),
										transitions={'succeeded': 'Explore and Detect', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pattern': 'pattern'})

			# x:537 y:438
			OperatableStateMachine.add('get_recovery_waypoint',
										GetRecoveryInfoState(service_topic='/trajectory_recovery_info'),
										transitions={'success': 'stuck_behavior', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})

			# x:661 y:317
			OperatableStateMachine.add('stuck_behavior',
										MoveToWaypointState(),
										transitions={'reached': 'Explore and Detect', 'failed': 'failed', 'update': 'Explore and Detect'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'update': Autonomy.Off},
										remapping={'waypoint': 'waypoint', 'victim': 'victim', 'speed': 'speed'})

			# x:757 y:210
			OperatableStateMachine.add('move_to_heat_source',
										MoveToWaypointState(),
										transitions={'reached': 'confirm_heat_source', 'failed': 'Explore and Detect', 'update': 'move_to_heat_source'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'update': Autonomy.Off},
										remapping={'waypoint': 'pose', 'victim': 'victim', 'speed': 'speed'})

			# x:759 y:79
			OperatableStateMachine.add('confirm_heat_source',
										ConfirmVictim(),
										transitions={'confirmed': 'Explore and Detect'},
										autonomy={'confirmed': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:513 y:204
			OperatableStateMachine.add('Explore and Detect',
										_sm_explore_and_detect_0,
										transitions={'finished': 'move_to_heat_source', 'failed': 'failed', 'stuck': 'get_recovery_waypoint'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'stuck': Autonomy.Inherit},
										remapping={'victim': 'victim', 'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
