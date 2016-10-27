#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_explore_and_search_victims')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from behavior_explore.explore_sm import ExploreSM
from hector_flexbe_states.detect_object import DetectObject
from hector_flexbe_states.discard_victim import DiscardVictim
from flexbe_states.operator_decision_state import OperatorDecisionState
from behavior_approach_victim.approach_victim_sm import ApproachVictimSM
from hector_flexbe_states.confirm_victim import ConfirmVictim
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu May 19 2016
@author: Elisa und Gabriel
'''
class ExploreandSearchVictimsSM(Behavior):
	'''
	Explore and search victims without arm movements.
	'''


	def __init__(self):
		super(ExploreandSearchVictimsSM, self).__init__()
		self.name = 'Explore and Search Victims'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(ExploreSM, 'Explore and Detect/Explore')
		self.add_behavior(ApproachVictimSM, 'Approach Victim')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:685 y:553
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.pose = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:345 y:73, x:350 y:149, x:496 y:339
		_sm_explore_and_detect_0 = ConcurrencyContainer(outcomes=['finished'], output_keys=['pose', 'victim'], conditions=[
										('finished', [('Explore', 'finished')]),
										('finished', [('Detect_Victim', 'found')])
										])

		with _sm_explore_and_detect_0:
			# x:77 y:65
			OperatableStateMachine.add('Explore',
										self.use_behavior(ExploreSM, 'Explore and Detect/Explore'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:83 y:145
			OperatableStateMachine.add('Detect_Victim',
										DetectObject(),
										transitions={'found': 'finished'},
										autonomy={'found': Autonomy.Off},
										remapping={'pose': 'pose', 'victim': 'victim'})



		with _state_machine:
			# x:40 y:52
			OperatableStateMachine.add('Explore and Detect',
										_sm_explore_and_detect_0,
										transitions={'finished': 'Approach Victim'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})

			# x:301 y:328
			OperatableStateMachine.add('Discard_Victim',
										DiscardVictim(),
										transitions={'discarded': 'Explore and Detect'},
										autonomy={'discarded': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:496 y:252
			OperatableStateMachine.add('Verify_Victim',
										OperatorDecisionState(outcomes=['confirmed','discarded','retry'], hint='Confirm or discard victim', suggestion='confirmed'),
										transitions={'confirmed': 'Confirm_Victim', 'discarded': 'Discard_Victim', 'retry': 'Approach Victim'},
										autonomy={'confirmed': Autonomy.High, 'discarded': Autonomy.Full, 'retry': Autonomy.Full})

			# x:491 y:54
			OperatableStateMachine.add('Approach Victim',
										self.use_behavior(ApproachVictimSM, 'Approach Victim'),
										transitions={'finished': 'Verify_Victim', 'failed': 'Verify_Victim'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})

			# x:301 y:178
			OperatableStateMachine.add('Confirm_Victim',
										ConfirmVictim(),
										transitions={'confirmed': 'Explore and Detect'},
										autonomy={'confirmed': Autonomy.Off},
										remapping={'victim': 'victim'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
