#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_search_victims_no_arm')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from behavior_exploration.exploration_sm import ExplorationSM
from hector_flexbe_states.detect_object import DetectObject
from hector_flexbe_states.confirm_victim import ConfirmVictim
from hector_flexbe_states.discard_victim import DiscardVictim
from behavior_explorationdriveto.explorationdriveto_sm import ExplorationDriveToSM
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu May 19 2016
@author: Elisa und Gabriel
'''
class SearchVictimsnoArmSM(Behavior):
	'''
	Explore and search victims without moving arm
	'''


	def __init__(self):
		super(SearchVictimsnoArmSM, self).__init__()
		self.name = 'Search Victims no Arm'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(ExplorationSM, 'ExplorationWithDetection/Exploration')
		self.add_behavior(ExplorationDriveToSM, 'ExplorationDriveTo')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		srdf = "hector_tracker_robot_moveit_config/config/taurob_tracker.srdf"
		arm_gripper_joints = ["arm_joint_%d"%i for i in range(5)]
		# x:685 y:553, x:911 y:41
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.group_name = 'arm_group'
		_state_machine.userdata.explore_speed = 0.2
		_state_machine.userdata.drive_to_speed = 0.2
		_state_machine.userdata.type = 'hotspot'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_explorationwithdetection_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['speed'], output_keys=['pose', 'victim'], conditions=[
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
			# x:159 y:82
			OperatableStateMachine.add('ExplorationWithDetection',
										_sm_explorationwithdetection_0,
										transitions={'finished': 'ExplorationDriveTo', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'speed': 'explore_speed', 'pose': 'pose', 'victim': 'victim'})

			# x:420 y:255
			OperatableStateMachine.add('Confirm_Victim',
										ConfirmVictim(),
										transitions={'confirmed': 'ExplorationWithDetection'},
										autonomy={'confirmed': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:412 y:338
			OperatableStateMachine.add('Discard_Victim',
										DiscardVictim(),
										transitions={'discarded': 'ExplorationWithDetection'},
										autonomy={'discarded': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:881 y:197
			OperatableStateMachine.add('ExplorationDriveTo',
										self.use_behavior(ExplorationDriveToSM, 'ExplorationDriveTo'),
										transitions={'finished': 'Decide_If_Victim', 'failed': 'Decide_If_Victim'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})

			# x:612 y:279
			OperatableStateMachine.add('Decide_If_Victim',
										OperatorDecisionState(outcomes=['confirmed','discarded','retry'], hint='Confirm or discard victim', suggestion='confirmed'),
										transitions={'confirmed': 'Confirm_Victim', 'discarded': 'Discard_Victim', 'retry': 'ExplorationDriveTo'},
										autonomy={'confirmed': Autonomy.High, 'discarded': Autonomy.Full, 'retry': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
