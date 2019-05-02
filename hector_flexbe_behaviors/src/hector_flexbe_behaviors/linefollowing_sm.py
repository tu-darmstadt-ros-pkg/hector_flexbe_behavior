#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_argos_states.get_path_state import GetPathState
from hector_flexbe_states.get_waypoint_from_array_state import GetWaypointFromArrayState
from hector_flexbe_states.switch_line_follower_direction import SwitchLineFollowerDirectionState
from flexbe_states.decision_state import DecisionState
from hector_flexbe_states.line_follower_state import LineFollowerState
from hector_flexbe_states.move_to_waypoint_state import MoveToWaypointState as hector_flexbe_states__MoveToWaypointState
from hector_flexbe_states.write_3d_map_state import Write3dMapState
from hector_flexbe_states.write_2d_map_state import Write2dMapState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Apr 24 2019
@author: Katrin Becker
'''
class LineFollowingSM(Behavior):
	'''
	Behavior for RoboCup Man3 Task (LineFollowing on a ramp)
	'''


	def __init__(self):
		super(LineFollowingSM, self).__init__()
		self.name = 'LineFollowing'

		# parameters of this behavior
		self.add_parameter('timeout_sec', 15)
		self.add_parameter('speed', 0.2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1012 y:445, x:1229 y:485
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.speed = self.speed
		_state_machine.userdata.camera_topic = '/front_rgbd_cam/color/image_rect_color'
		_state_machine.userdata.drive_backwards = False
		_state_machine.userdata.nextWaypoint = ''
		_state_machine.userdata.waypoints = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:990 y:286, x:1149 y:264
		_sm_followline_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['nextWaypoint', 'speed', 'drive_backwards', 'camera_topic'])

		with _sm_followline_0:
			# x:230 y:85
			OperatableStateMachine.add('LineFollower',
										LineFollowerState(timeout_sec=self.timeout_sec),
										transitions={'reached': 'hasWaypoint?', 'failed': 'hasWaypoint?'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'camera_topic': 'camera_topic', 'drive_backwards': 'drive_backwards', 'speed': 'speed'})

			# x:550 y:87
			OperatableStateMachine.add('hasWaypoint?',
										DecisionState(outcomes=['waypoint', 'no_waypoint'], conditions=lambda x: 'waypoint' if x else 'no_waypoint'),
										transitions={'waypoint': 'MoveToWaypoint', 'no_waypoint': 'finished'},
										autonomy={'waypoint': Autonomy.Off, 'no_waypoint': Autonomy.Off},
										remapping={'input_value': 'nextWaypoint'})

			# x:899 y:96
			OperatableStateMachine.add('MoveToWaypoint',
										hector_flexbe_states__MoveToWaypointState(position_tolerance=0.2, angle_tolerance=3, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=False),
										transitions={'reached': 'finished', 'failed': 'MoveToWaypoint', 'stuck': 'MoveToWaypoint'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'waypoint': 'nextWaypoint', 'speed': 'speed'})



		with _state_machine:
			# x:116 y:61
			OperatableStateMachine.add('getWaypoints',
										GetPathState(pathTopic='/path_to_follow'),
										transitions={'succeeded': 'haveWaypoints?', 'failed': 'haveWaypoints?'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints2': 'waypoints'})

			# x:569 y:226
			OperatableStateMachine.add('getCurrentWaypoint',
										GetWaypointFromArrayState(position=0),
										transitions={'succeeded': 'FollowLine', 'empty': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoint': 'nextWaypoint'})

			# x:1342 y:62
			OperatableStateMachine.add('SwitchDirection',
										SwitchLineFollowerDirectionState(camera_topic_forwards='/front_rgbd_cam/color/image_rect_color', camera_topic_backwards='/back_rgbd_cam/color/image_rect_color'),
										transitions={'finished': 'reachedLower'},
										autonomy={'finished': Autonomy.Off},
										remapping={'reverse': 'drive_backwards', 'camera_topic': 'camera_topic'})

			# x:369 y:58
			OperatableStateMachine.add('haveWaypoints?',
										DecisionState(outcomes=['waypoints', 'no_waypoints'], conditions=lambda x: 'waypoints' if x else 'no_waypoints'),
										transitions={'waypoints': 'getCurrentWaypoint', 'no_waypoints': 'FollowLine'},
										autonomy={'waypoints': Autonomy.Off, 'no_waypoints': Autonomy.Off},
										remapping={'input_value': 'waypoints'})

			# x:875 y:169
			OperatableStateMachine.add('FollowLine',
										_sm_followline_0,
										transitions={'finished': 'write_3d_map', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'nextWaypoint': 'nextWaypoint', 'speed': 'speed', 'drive_backwards': 'drive_backwards', 'camera_topic': 'camera_topic'})

			# x:974 y:64
			OperatableStateMachine.add('reachedLower',
										DecisionState(outcomes=['yes', 'no'], conditions=lambda x: 'no' if x else 'yes'),
										transitions={'yes': 'finished', 'no': 'haveWaypoints?'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off},
										remapping={'input_value': 'drive_backwards'})

			# x:1152 y:173
			OperatableStateMachine.add('write_3d_map',
										Write3dMapState(service_topic='/worldmodel_main/save_map', save_path='/octomaps/linefollowing'),
										transitions={'success': 'write_2d_map', 'failed': 'write_2d_map'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1376 y:174
			OperatableStateMachine.add('write_2d_map',
										Write2dMapState(writer_topic='/syscommand'),
										transitions={'success': 'SwitchDirection', 'failed': 'SwitchDirection'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
