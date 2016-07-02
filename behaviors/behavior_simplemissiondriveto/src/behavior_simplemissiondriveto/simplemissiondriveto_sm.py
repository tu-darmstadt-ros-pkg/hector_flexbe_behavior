#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_simplemissiondriveto')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.move_to_fixed_waypoint import MoveToFixedWaypoint
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Tue Jun 07 2016
@author: Gabriel Elisa
'''
class SimpleMissionDriveToSM(Behavior):
	'''
	Drive to exact given point
	'''


	def __init__(self):
		super(SimpleMissionDriveToSM, self).__init__()
		self.name = 'SimpleMissionDriveTo'

		# parameters of this behavior
		self.add_parameter('speed', 0.3)
		self.add_parameter('allow_backwards', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pose'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.speed = self.speed

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:213 y:74
			OperatableStateMachine.add('Move_To',
										MoveToFixedWaypoint(allow_backwards=self.allow_backwards),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pose', 'speed': 'speed'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
