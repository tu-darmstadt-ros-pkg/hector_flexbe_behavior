#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_simplemission')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from behavior_simplemissioninitialize.simplemissioninitialize_sm import SimpleMissionInitializeSM
from hector_flexbe_states.MarkPoint import MarkPoint
from hector_flexbe_states.Mapping import Mapping
from hector_flexbe_states.drive_to_new import Drive_to_new
from behavior_simplemissiondoaction.simplemissiondoaction_sm import SimpleMissionDoActionSM
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu Jun 02 2016
@author: Gabriel
'''
class SimpleMissionSM(Behavior):
	'''
	Simple mission structure
	'''


	def __init__(self):
		super(SimpleMissionSM, self).__init__()
		self.name = 'SimpleMission'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SimpleMissionInitializeSM, 'SimpleMissionInitialize')
		self.add_behavior(SimpleMissionDoActionSM, 'SimpleMissionDoAction')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.startPoint = PoseStamped()
		_state_machine.userdata.endPoint = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:31 y:51
			OperatableStateMachine.add('SimpleMissionInitialize',
										self.use_behavior(SimpleMissionInitializeSM, 'SimpleMissionInitialize'),
										transitions={'finished': 'Operator_Drive', 'failed': 'Operator_Drive'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'startPoint': 'startPoint'})

			# x:496 y:56
			OperatableStateMachine.add('Endpoint',
										MarkPoint(),
										transitions={'succeeded': 'DeactivateMapping'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'endPoint'})

			# x:672 y:55
			OperatableStateMachine.add('DeactivateMapping',
										Mapping(),
										transitions={'succeeded': 'SimpleMissionDoAction'},
										autonomy={'succeeded': Autonomy.Off})

			# x:292 y:232
			OperatableStateMachine.add('Drive_to_Start',
										Drive_to_new(),
										transitions={'succeeded': 'Drive_to_End'},
										autonomy={'succeeded': Autonomy.High},
										remapping={'pose': 'startPoint'})

			# x:632 y:227
			OperatableStateMachine.add('SimpleMissionDoAction',
										self.use_behavior(SimpleMissionDoActionSM, 'SimpleMissionDoAction'),
										transitions={'finished': 'Drive_to_Start', 'failed': 'Drive_to_Start'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:291 y:56
			OperatableStateMachine.add('Operator_Drive',
										WaitState(wait_time=5),
										transitions={'done': 'Endpoint'},
										autonomy={'done': Autonomy.High})

			# x:463 y:147
			OperatableStateMachine.add('Drive_to_End',
										Drive_to_new(),
										transitions={'succeeded': 'SimpleMissionDoAction'},
										autonomy={'succeeded': Autonomy.High},
										remapping={'pose': 'endPoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
