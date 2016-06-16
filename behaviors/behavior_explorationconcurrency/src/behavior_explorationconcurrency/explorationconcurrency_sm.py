#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_explorationconcurrency')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from behavior_newexp.newexp_sm import NewExpSM
from hector_flexbe_states.object_detection import Object_Detection
from flexbe_states.wait_state import WaitState
from hector_flexbe_states.victim_confirmation import Victim_Confirmation
from behavior_explorationdriveto.explorationdriveto_sm import ExplorationDriveToSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu May 19 2016
@author: Elisa Gabriel
'''
class ExplorationConcurrencySM(Behavior):
	'''
	Concurrency Test for Exploration
	'''


	def __init__(self):
		super(ExplorationConcurrencySM, self).__init__()
		self.name = 'ExplorationConcurrency'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(NewExpSM, 'Container/NewExp')
		self.add_behavior(ExplorationDriveToSM, 'ExplorationDriveTo')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:129 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['pose', 'victim'], conditions=[
										('finished', [('NewExp', 'finished')]),
										('failed', [('NewExp', 'failed')]),
										('finished', [('Detect', 'continue')]),
										('failed', [('Detect', 'found')])
										])

		with _sm_container_0:
			# x:30 y:40
			OperatableStateMachine.add('NewExp',
										self.use_behavior(NewExpSM, 'Container/NewExp'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:210 y:40
			OperatableStateMachine.add('Detect',
										Object_Detection(),
										transitions={'continue': 'finished', 'found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'found': Autonomy.Off},
										remapping={'pose': 'pose', 'victim': 'victim'})



		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'ExplorationDriveTo'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})

			# x:125 y:159
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=5),
										transitions={'done': 'Container'},
										autonomy={'done': Autonomy.Off})

			# x:271 y:197
			OperatableStateMachine.add('Confirm',
										Victim_Confirmation(),
										transitions={'discard': 'failed', 'confirm': 'Wait'},
										autonomy={'discard': Autonomy.Off, 'confirm': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:644 y:179
			OperatableStateMachine.add('ExplorationDriveTo',
										self.use_behavior(ExplorationDriveToSM, 'ExplorationDriveTo'),
										transitions={'finished': 'Confirm', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
