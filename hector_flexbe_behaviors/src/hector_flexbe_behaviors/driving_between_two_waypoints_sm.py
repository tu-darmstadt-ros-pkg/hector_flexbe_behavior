#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_behaviors.set_points_sm import SetPointsSM
from flexbe_behaviors.simplemissiondriveto_sm import SimpleMissionDriveToSM
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu Jun 02 2016
@author: Gabriel
'''
class DrivingbetweentwoWaypointsSM(Behavior):
	'''
	Simple mission structure
	'''


	def __init__(self):
		super(DrivingbetweentwoWaypointsSM, self).__init__()
		self.name = 'Driving between two Waypoints'

		# parameters of this behavior
		self.add_parameter('speed', 0.1)
		self.add_parameter('allow_backwards', False)

		# references to used behaviors
		self.add_behavior(SetPointsSM, 'Set Points')
		self.add_behavior(SimpleMissionDriveToSM, 'Drive To Start')
		self.add_behavior(SimpleMissionDriveToSM, 'Drive To End')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:52 y:481, x:134 y:482
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.startPoint = PoseStamped()
		_state_machine.userdata.endPoint = PoseStamped()
		_state_machine.userdata.switchFalse = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:32 y:58
			OperatableStateMachine.add('Set Points',
										self.use_behavior(SetPointsSM, 'Set Points'),
										transitions={'finished': 'Operator_Drive'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'startPoint': 'startPoint', 'endPoint': 'endPoint'})

			# x:68 y:172
			OperatableStateMachine.add('Drive To Start',
										self.use_behavior(SimpleMissionDriveToSM, 'Drive To Start'),
										transitions={'finished': 'Drive To End', 'failed': 'Drive To Start'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'startPoint'})

			# x:287 y:59
			OperatableStateMachine.add('Operator_Drive',
										OperatorDecisionState(outcomes=['toStart', 'toEnd'], hint='Operator Drive', suggestion=None),
										transitions={'toStart': 'Drive To Start', 'toEnd': 'Drive To End'},
										autonomy={'toStart': Autonomy.Off, 'toEnd': Autonomy.Off})

			# x:468 y:172
			OperatableStateMachine.add('Drive To End',
										self.use_behavior(SimpleMissionDriveToSM, 'Drive To End'),
										transitions={'finished': 'Drive To Start', 'failed': 'Drive To End'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'endPoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
