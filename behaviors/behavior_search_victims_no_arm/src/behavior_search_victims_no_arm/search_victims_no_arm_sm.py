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
from hector_flexbe_states.discard_victim import DiscardVictim
from behavior_explorationdriveto.explorationdriveto_sm import ExplorationDriveToSM
from flexbe_states.operator_decision_state import OperatorDecisionState
from hector_flexbe_states.confirm_victim import ConfirmVictim
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
		self.add_behavior(ExplorationSM, 'Explore and Detect/Exploration')
		self.add_behavior(ExplorationDriveToSM, 'ExplorationDriveTo')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:685 y:553, x:911 y:41
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:322, x:130 y:322, x:230 y:322, x:330 y:322
		_sm_explore_and_detect_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['victim', 'pose'], conditions=[
										('finished', [('Exploration', 'finished')]),
										('finished', [('Detect_Victim', 'found')])
										])

		with _sm_explore_and_detect_0:
			# x:59 y:60
			OperatableStateMachine.add('Exploration',
										self.use_behavior(ExplorationSM, 'Explore and Detect/Exploration'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:223 y:68
			OperatableStateMachine.add('Detect_Victim',
										DetectObject(),
										transitions={'found': 'finished'},
										autonomy={'found': Autonomy.Off},
										remapping={'pose': 'pose', 'victim': 'victim'})



		with _state_machine:
			# x:98 y:55
			OperatableStateMachine.add('Explore and Detect',
										_sm_explore_and_detect_0,
										transitions={'finished': 'ExplorationDriveTo', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'victim': 'victim', 'pose': 'pose'})

			# x:412 y:338
			OperatableStateMachine.add('Discard_Victim',
										DiscardVictim(),
										transitions={'discarded': 'Explore and Detect'},
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

			# x:420 y:255
			OperatableStateMachine.add('Confirm_Victim',
										ConfirmVictim(),
										transitions={'confirmed': 'Explore and Detect'},
										autonomy={'confirmed': Autonomy.Off},
										remapping={'victim': 'victim'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
