#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_behaviors.explore_sm import ExploreSM
from hector_flexbe_states.detect_object import DetectObject
from hector_flexbe_states.discard_victim import DiscardVictim
from flexbe_states.operator_decision_state import OperatorDecisionState
from hector_flexbe_states.confirm_victim import ConfirmVictim
from flexbe_behaviors.approach_victim_sm import ApproachVictimSM
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
		self.add_behavior(ExploreSM, 'Explore and Detect/Explore')
		self.add_behavior(ApproachVictimSM, 'Approach Victim')

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

		# x:212 y:333, x:130 y:322, x:54 y:320, x:330 y:322
		_sm_explore_and_detect_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['victim', 'pose'], conditions=[
										('finished', [('Detect_Victim', 'found')]),
										('finished', [('Explore', 'finished')])
										])

		with _sm_explore_and_detect_0:
			# x:54 y:58
			OperatableStateMachine.add('Explore',
										self.use_behavior(ExploreSM, 'Explore and Detect/Explore'),
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
										transitions={'finished': 'Approach Victim', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'victim': 'victim', 'pose': 'pose'})

			# x:412 y:338
			OperatableStateMachine.add('Discard_Victim',
										DiscardVictim(),
										transitions={'discarded': 'Explore and Detect'},
										autonomy={'discarded': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:612 y:279
			OperatableStateMachine.add('Decide_If_Victim',
										OperatorDecisionState(outcomes=['confirmed','discarded','retry'], hint='Confirm or discard victim', suggestion='confirmed'),
										transitions={'confirmed': 'Confirm_Victim', 'discarded': 'Discard_Victim', 'retry': 'Approach Victim'},
										autonomy={'confirmed': Autonomy.High, 'discarded': Autonomy.Full, 'retry': Autonomy.Full})

			# x:420 y:255
			OperatableStateMachine.add('Confirm_Victim',
										ConfirmVictim(),
										transitions={'confirmed': 'Explore and Detect'},
										autonomy={'confirmed': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:798 y:98
			OperatableStateMachine.add('Approach Victim',
										self.use_behavior(ApproachVictimSM, 'Approach Victim'),
										transitions={'finished': 'Decide_If_Victim', 'failed': 'Decide_If_Victim'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
