#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.detect_object_new import DetectObjectNew
from hector_flexbe_states.move_arm_dyn_state import MoveArmDynState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Fri Jun 24 2016
@author: Gabriel und Elisa
'''
class SimpleMissionStartcheckSM(Behavior):
	'''
	Perform the Startcheck at the beginning of each Preliminary Task
	'''


	def __init__(self):
		super(SimpleMissionStartcheckSM, self).__init__()
		self.name = 'SimpleMissionStartcheck'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.startCheckPosition = PoseStamped()
		_state_machine.userdata.startCheckID = ''
		_state_machine.userdata.objectType = 'start_check_pipe'
		_state_machine.userdata.objectState = 0
		_state_machine.userdata.joint_config = [0,0,0,0]
		_state_machine.userdata.group_name = 'arm_group'
		_state_machine.userdata.armType = 'pipes'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('DetectObject',
										DetectObjectNew(),
										transitions={'found': 'MoveArmPipe'},
										autonomy={'found': Autonomy.Off},
										remapping={'type': 'objectType', 'state': 'objectState', 'pose': 'startCheckPosition', 'object_id': 'startCheckID'})

			# x:326 y:64
			OperatableStateMachine.add('MoveArmPipe',
										MoveArmDynState(),
										transitions={'reached': 'finished', 'sampling_failed': 'MoveArmPipe', 'planning_failed': 'MoveArmPipe', 'control_failed': 'MoveArmPipe'},
										autonomy={'reached': Autonomy.Off, 'sampling_failed': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'object_pose': 'startCheckPosition', 'object_type': 'armType', 'object_id': 'startCheckID'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
