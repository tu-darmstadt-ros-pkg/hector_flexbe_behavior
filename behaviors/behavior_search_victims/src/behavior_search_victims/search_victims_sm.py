#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_search_victims')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.move_arm_state import MoveArmState
from flexbe_states.wait_state import WaitState
from behavior_explorationdriveto.explorationdriveto_sm import ExplorationDriveToSM
from hector_flexbe_states.confirm_victim import ConfirmVictim
from hector_flexbe_states.discard_victim import DiscardVictim
from hector_flexbe_states.Decide_If_Victim import DecideIfVictim
from behavior_exploration.exploration_sm import ExplorationSM
from hector_flexbe_states.detect_object import DetectObject
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu May 19 2016
@author: Elisa und Gabriel
'''
class SearchVictimsSM(Behavior):
	'''
	Explore and search victims
	'''


	def __init__(self):
		super(SearchVictimsSM, self).__init__()
		self.name = 'Search Victims'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(ExplorationDriveToSM, 'ExplorationDriveTo')
		self.add_behavior(ExplorationSM, 'ExplorationWithDetection/Exploration')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:43 y:428
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.joint_config = [0, 0, 0, 0]
		_state_machine.userdata.group_name = 'arm_group'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_explorationwithdetection_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['pose', 'victim'], conditions=[
										('finished', [('Exploration', 'finished')]),
										('failed', [('Exploration', 'failed')]),
										('finished', [('Detect_Object', 'found')])
										])

		with _sm_explorationwithdetection_0:
			# x:40 y:44
			OperatableStateMachine.add('Exploration',
										self.use_behavior(ExplorationSM, 'ExplorationWithDetection/Exploration'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:346 y:59
			OperatableStateMachine.add('Detect_Object',
										DetectObject(),
										transitions={'found': 'finished'},
										autonomy={'found': Autonomy.Off},
										remapping={'pose': 'pose', 'victim': 'victim'})



		with _state_machine:
			# x:84 y:49
			OperatableStateMachine.add('SetInitialArmState',
										MoveArmState(),
										transitions={'reached': 'ExplorationWithDetection', 'planning_failed': 'SetInitialArmState', 'control_failed': 'SetInitialArmState'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.High, 'control_failed': Autonomy.High},
										remapping={'joint_config': 'joint_config', 'group_name': 'group_name'})

			# x:125 y:149
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=5),
										transitions={'done': 'ExplorationWithDetection'},
										autonomy={'done': Autonomy.Off})

			# x:798 y:180
			OperatableStateMachine.add('ExplorationDriveTo',
										self.use_behavior(ExplorationDriveToSM, 'ExplorationDriveTo'),
										transitions={'finished': 'Decide_If_Victim', 'failed': 'Decide_If_Victim'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})

			# x:365 y:197
			OperatableStateMachine.add('Confirm_Victim',
										ConfirmVictim(),
										transitions={'confirmed': 'Wait'},
										autonomy={'confirmed': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:371 y:308
			OperatableStateMachine.add('Discard_Victim',
										DiscardVictim(),
										transitions={'discarded': 'Wait'},
										autonomy={'discarded': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:605 y:238
			OperatableStateMachine.add('Decide_If_Victim',
										DecideIfVictim(),
										transitions={'confirm': 'Confirm_Victim', 'discard': 'Discard_Victim', 'retry': 'ExplorationDriveTo'},
										autonomy={'confirm': Autonomy.Off, 'discard': Autonomy.Off, 'retry': Autonomy.Off})

			# x:400 y:34
			OperatableStateMachine.add('ExplorationWithDetection',
										_sm_explorationwithdetection_0,
										transitions={'finished': 'ExplorationDriveTo', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
