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
from flexbe_argos_states.move_to_waypoint_state import MoveToWaypointState
from hector_flexbe_states.confirm_victim import ConfirmVictim
from hector_flexbe_behaviors.rc_exp5_sm import rc_exp5SM
from hector_flexbe_states.detect_object import DetectObject
from hector_flexbe_states.wait_for_object_confirmation_state import WaitForObjectConfirmationState
from hector_flexbe_states.discard_victim import DiscardVictim
from flexbe_argos_states.look_at_waypoint_state import LookAtWaypoint
from hector_flexbe_states.drive_straight_state import DriveStraightState
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
		# x:605 y:434, x:454 y:399
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pattern = 'look_around'
		_state_machine.userdata.speed = 0.2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365
		_sm_explore_and_detect_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'stuck', 'detected'], output_keys=['victim', 'pose'], conditions=[
										('detected', [('find_object', 'found')]),
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
			OperatableStateMachine.add('find_object',
										DetectObject(),
										transitions={'found': 'detected'},
										autonomy={'found': Autonomy.Off},
										remapping={'pose': 'pose', 'victim': 'victim'})



		with _state_machine:
			# x:311 y:201
			OperatableStateMachine.add('start_head_pattern',
										LookAtPattern(),
										transitions={'succeeded': 'Explore and Detect', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pattern': 'pattern'})

			# x:602 y:465
			OperatableStateMachine.add('get_recovery_waypoint',
										GetRecoveryInfoState(service_topic='/trajectory_recovery_info'),
										transitions={'success': 'stuck_behavior', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})

			# x:698 y:344
			OperatableStateMachine.add('stuck_behavior',
										MoveToWaypointState(desired_speed=0.2, position_tolerance=0, angle_tolerance=0, rotate_to_goal=0, reverse_allowed=True),
										transitions={'reached': 'Explore and Detect', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})

			# x:502 y:269
			OperatableStateMachine.add('confirm_object',
										ConfirmVictim(),
										transitions={'confirmed': 'start_head_pattern'},
										autonomy={'confirmed': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:551 y:201
			OperatableStateMachine.add('Explore and Detect',
										_sm_explore_and_detect_0,
										transitions={'finished': 'finished', 'failed': 'failed', 'stuck': 'get_recovery_waypoint', 'detected': 'stop_robot'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'stuck': Autonomy.Inherit, 'detected': Autonomy.Inherit},
										remapping={'victim': 'victim', 'pose': 'pose'})

			# x:648 y:406
			OperatableStateMachine.add('wait_for_confirmation',
										WaitForObjectConfirmationState(),
										transitions={'confirmed': 'confirm_object', 'rejected': 'discard_object'},
										autonomy={'confirmed': Autonomy.Off, 'rejected': Autonomy.Off},
										remapping={'object_id': 'victim'})

			# x:461 y:337
			OperatableStateMachine.add('discard_object',
										DiscardVictim(),
										transitions={'discarded': 'start_head_pattern'},
										autonomy={'discarded': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:841 y:361
			OperatableStateMachine.add('look_at_object',
										LookAtWaypoint(action_topic='/pan_tilt_sensor_head_joint_control/look_at', frame_id='world', no_tracking=False),
										transitions={'reached': 'wait_for_confirmation', 'failed': 'wait_for_confirmation'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pose'})

			# x:748 y:188
			OperatableStateMachine.add('stop_robot',
										DriveStraightState(desired_speed=0.2, duration=0.2, reverse=False, velocity_topic='/cmd_vel'),
										transitions={'reached': 'look_at_object', 'failed': 'look_at_object'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
