#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.mission_initialisation_state import MissionInitialisationState
from hector_flexbe_behaviors.explore_mission_robocup_sm import ExploreMissionRobocupSM
from hector_flexbe_behaviors.waypoint_mission_sm import WaypointmissionSM
from flexbe_states.wait_state import WaitState
from hector_flexbe_states.mission_decision_state import MissionDecisionState
from hector_flexbe_behaviors.linefollowing_sm import LineFollowingSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 15 2019
@author: Gabriel Huettenberger
'''
class GenericStaticTestMethodSM(Behavior):
	'''
	Behavior for executing a generic test method on a known map
	'''


	def __init__(self):
		super(GenericStaticTestMethodSM, self).__init__()
		self.name = 'Generic Static Test Method'

		# parameters of this behavior
		self.add_parameter('hazmatEnabled', False)
		self.add_parameter('traversabilityMap', False)
		self.add_parameter('waypointFollowing', False)
		self.add_parameter('exploration', False)
		self.add_parameter('roughTerrain', False)
		self.add_parameter('repeat', True)
		self.add_parameter('specialFunctionality', '')
		self.add_parameter('speed', 0.2)

		# references to used behaviors
		self.add_behavior(ExploreMissionRobocupSM, 'Explore Mission Robocup')
		self.add_behavior(WaypointmissionSM, 'Waypoint mission')
		self.add_behavior(LineFollowingSM, 'LineFollowing')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.hazmatEnabled = self.hazmatEnabled
		_state_machine.userdata.traversabilityMap = self.traversabilityMap
		_state_machine.userdata.waypointFollowing = self.waypointFollowing
		_state_machine.userdata.exploration = self.exploration
		_state_machine.userdata.roughTerrain = self.roughTerrain
		_state_machine.userdata.specialFunctionality = self.specialFunctionality
		_state_machine.userdata.repeat = self.repeat
		_state_machine.userdata.speed = self.speed
		_state_machine.userdata.reexplore_time = 5

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:87 y:53
			OperatableStateMachine.add('initalise_parameter',
										MissionInitialisationState(),
										transitions={'done': 'select_mission', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'hazmatEnabled': 'hazmatEnabled', 'traversabilityMap': 'traversabilityMap', 'roughTerrain': 'roughTerrain'})

			# x:552 y:365
			OperatableStateMachine.add('Explore Mission Robocup',
										self.use_behavior(ExploreMissionRobocupSM, 'Explore Mission Robocup'),
										transitions={'finished': 'WRITE_RESULTS_DUMMY2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'reexplore_time': 'reexplore_time', 'speed': 'speed'})

			# x:547 y:169
			OperatableStateMachine.add('Waypoint mission',
										self.use_behavior(WaypointmissionSM, 'Waypoint mission'),
										transitions={'finished': 'WRITE_RESULTS_DUMMY', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'speed': 'speed', 'reexplore_time': 'reexplore_time'})

			# x:760 y:262
			OperatableStateMachine.add('WRITE_RESULTS_DUMMY',
										WaitState(wait_time=1),
										transitions={'done': 'Waypoint mission'},
										autonomy={'done': Autonomy.Off})

			# x:755 y:444
			OperatableStateMachine.add('WRITE_RESULTS_DUMMY2',
										WaitState(wait_time=1),
										transitions={'done': 'Explore Mission Robocup'},
										autonomy={'done': Autonomy.Off})

			# x:305 y:90
			OperatableStateMachine.add('select_mission',
										MissionDecisionState(),
										transitions={'followMission': 'Waypoint mission', 'exploreMission': 'Explore Mission Robocup', 'combinedMission': 'select_mission', 'followLineMission': 'LineFollowing', 'failed': 'failed'},
										autonomy={'followMission': Autonomy.Off, 'exploreMission': Autonomy.Off, 'combinedMission': Autonomy.Off, 'followLineMission': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exploration': 'exploration', 'waypointFollowing': 'waypointFollowing', 'specialFunctionality': 'specialFunctionality'})

			# x:564 y:45
			OperatableStateMachine.add('LineFollowing',
										self.use_behavior(LineFollowingSM, 'LineFollowing'),
										transitions={'finished': 'WRITE_RESULTS_DUMMY3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:777 y:57
			OperatableStateMachine.add('WRITE_RESULTS_DUMMY3',
										WaitState(wait_time=1),
										transitions={'done': 'LineFollowing'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
