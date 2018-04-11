#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.create_path import CreatePath
from hector_flexbe_states.invert_path import InvertPath
from hector_flexbe_states.move_along_path import MoveAlongPath
from hector_flexbe_states.sparse_path import SparsePath
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu Jun 02 2016
@author: Gabriel, Elisa
'''
class PathDriveMissionSM(Behavior):
	'''
	Robot moves along a given path
	'''


	def __init__(self):
		super(PathDriveMissionSM, self).__init__()
		self.name = 'PathDriveMission'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:52 y:481, x:179 y:505
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.speed = 0.2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:169 y:61
			OperatableStateMachine.add('Create_Path',
										CreatePath(),
										transitions={'succeeded': 'Invert_Path', 'retry': 'Create_Path'},
										autonomy={'succeeded': Autonomy.Off, 'retry': Autonomy.Off},
										remapping={'path': 'path'})

			# x:309 y:56
			OperatableStateMachine.add('Invert_Path',
										InvertPath(),
										transitions={'reached': 'Sparse_Path', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'path': 'path'})

			# x:670 y:162
			OperatableStateMachine.add('Move_Along_Path',
										MoveAlongPath(),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'path': 'path', 'speed': 'speed'})

			# x:482 y:64
			OperatableStateMachine.add('Sparse_Path',
										SparsePath(max_dist=.2, max_angle=.2, min_dist=.1),
										transitions={'done': 'Move_Along_Path'},
										autonomy={'done': Autonomy.Off},
										remapping={'path': 'path'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
