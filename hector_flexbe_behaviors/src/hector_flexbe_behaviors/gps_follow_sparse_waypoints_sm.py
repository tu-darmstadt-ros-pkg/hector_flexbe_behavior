#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_argos_states.import_gps_from_csv_state import ImportGpsFromCsvState
from flexbe_argos_states.parse_gps_waypoints_state import ParseGPSWaypointsState
from hector_flexbe_behaviors.follow_waypoints_sm import FollowwaypointsSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Mon Jul 17 2017
@author: Gabriel Huettenberger
'''
class GPSFollowsparsewaypointsSM(Behavior):
	'''
	Follow waypoints given by GPS coordinates
	'''


	def __init__(self):
		super(GPSFollowsparsewaypointsSM, self).__init__()
		self.name = 'GPS Follow sparse waypoints'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(FollowwaypointsSM, 'Follow waypoints')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.current_waypoint = PoseStamped

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:52 y:39
			OperatableStateMachine.add('import waypoints',
										ImportGpsFromCsvState(waypointString='/home/gabriel/Downloads/example_gps.csv', pattern=''),
										transitions={'succeeded': 'parse waypoints'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoints2': 'waypoints2'})

			# x:308 y:42
			OperatableStateMachine.add('parse waypoints',
										ParseGPSWaypointsState(marker_lifetime=4, pattern='*'),
										transitions={'succeeded': 'parse waypoints', 'start': 'Follow waypoints'},
										autonomy={'succeeded': Autonomy.Off, 'start': Autonomy.Off},
										remapping={'waypoints': 'waypoints'})

			# x:309 y:188
			OperatableStateMachine.add('Follow waypoints',
										self.use_behavior(FollowwaypointsSM, 'Follow waypoints'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
