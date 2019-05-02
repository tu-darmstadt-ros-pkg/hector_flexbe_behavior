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
from hector_flexbe_states.mission_decision_state import MissionDecisionState
from hector_flexbe_behaviors.linefollowing_sm import LineFollowingSM
from hector_flexbe_states.write_3d_map_state import Write3dMapState
from hector_flexbe_states.write_2d_map_state import Write2dMapState
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
		# x:30 y:365, x:130 y:309
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
										remapping={'hazmatEnabled': 'hazmatEnabled', 'traversabilityMap': 'traversabilityMap', 'roughTerrain': 'roughTerrain', 'exploration': 'exploration'})

			# x:552 y:365
			OperatableStateMachine.add('Explore Mission Robocup',
										self.use_behavior(ExploreMissionRobocupSM, 'Explore Mission Robocup'),
										transitions={'finished': 'write_3d_map_3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'reexplore_time': 'reexplore_time', 'speed': 'speed'})

			# x:557 y:198
			OperatableStateMachine.add('Waypoint mission',
										self.use_behavior(WaypointmissionSM, 'Waypoint mission'),
										transitions={'finished': 'write_3d_map_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'speed': 'speed', 'reexplore_time': 'reexplore_time'})

			# x:305 y:90
			OperatableStateMachine.add('select_mission',
										MissionDecisionState(),
										transitions={'followMission': 'Waypoint mission', 'exploreMission': 'Explore Mission Robocup', 'combinedMission': 'select_mission', 'followLineMission': 'LineFollowing', 'failed': 'failed'},
										autonomy={'followMission': Autonomy.Off, 'exploreMission': Autonomy.Off, 'combinedMission': Autonomy.Off, 'followLineMission': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'exploration': 'exploration', 'waypointFollowing': 'waypointFollowing', 'specialFunctionality': 'specialFunctionality'})

			# x:564 y:45
			OperatableStateMachine.add('LineFollowing',
										self.use_behavior(LineFollowingSM, 'LineFollowing'),
										transitions={'finished': 'write_3d_map', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:857 y:11
			OperatableStateMachine.add('write_3d_map',
										Write3dMapState(service_topic='/worldmodel_main/save_map', save_path='/octomaps/linefollowing'),
										transitions={'success': 'write_2d_map', 'failed': 'write_2d_map'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:862 y:162
			OperatableStateMachine.add('write_3d_map_2',
										Write3dMapState(service_topic='/worldmodel_main/save_map', save_path='/octomaps/waypoints'),
										transitions={'success': 'write_2d_map_2', 'failed': 'write_2d_map_2'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:864 y:319
			OperatableStateMachine.add('write_3d_map_3',
										Write3dMapState(service_topic='/worldmodel_main/save_map', save_path='/octomaps/explore'),
										transitions={'success': 'write_2d_map_3', 'failed': 'write_2d_map_3'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:860 y:79
			OperatableStateMachine.add('write_2d_map',
										Write2dMapState(writer_topic='/syscommand'),
										transitions={'success': 'LineFollowing', 'failed': 'LineFollowing'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:860 y:234
			OperatableStateMachine.add('write_2d_map_2',
										Write2dMapState(writer_topic='/syscommand'),
										transitions={'success': 'Waypoint mission', 'failed': 'Waypoint mission'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:864 y:381
			OperatableStateMachine.add('write_2d_map_3',
										Write2dMapState(writer_topic='/syscommand'),
										transitions={'success': 'Explore Mission Robocup', 'failed': 'Explore Mission Robocup'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
