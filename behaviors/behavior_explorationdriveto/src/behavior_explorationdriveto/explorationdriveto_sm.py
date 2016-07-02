#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_explorationdriveto')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.LookAtWaypoint import LookAtWaypoint
from hector_flexbe_states.move_to_waypoint_state import MoveToWaypointState
from hector_flexbe_states.get_object_pose_state import GetObjectPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Tue Jun 07 2016
@author: Gabriel Elisa
'''
class ExplorationDriveToSM(Behavior):
	'''
	Drive to exact given point
	'''


	def __init__(self):
		super(ExplorationDriveToSM, self).__init__()
		self.name = 'ExplorationDriveTo'

		# parameters of this behavior
		self.add_parameter('speed', 0.1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pose', 'victim'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.victim = ''
		_state_machine.userdata.speed = self.speed

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:175 y:79
			OperatableStateMachine.add('Look_At',
										LookAtWaypoint(),
										transitions={'reached': 'Move_To', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pose'})

			# x:445 y:91
			OperatableStateMachine.add('Move_To',
										MoveToWaypointState(),
										transitions={'reached': 'finished', 'failed': 'failed', 'update': 'GetVictimPose'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'update': Autonomy.Off},
										remapping={'waypoint': 'pose', 'victim': 'victim', 'speed': 'speed'})

			# x:530 y:247
			OperatableStateMachine.add('GetVictimPose',
										GetObjectPoseState(),
										transitions={'done': 'Move_To', 'unknown': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'unknown': Autonomy.Off, 'not_available': Autonomy.Off},
										remapping={'object_id': 'victim', 'object_pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
