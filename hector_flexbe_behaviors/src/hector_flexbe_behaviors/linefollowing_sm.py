#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_argos_states.get_path_from_service_state import GetPathFromServiceState as flexbe_argos_states__GetPathFromServiceState
from hector_flexbe_states.switch_line_follower_direction import SwitchLineFollowerDirectionState
from flexbe_states.decision_state import DecisionState
from hector_flexbe_states.write_3d_map_state import Write3dMapState
from hector_flexbe_states.write_2d_map_state import Write2dMapState
from hector_flexbe_states.get_waypoint_from_array_state import GetWaypointFromArrayState
from hector_flexbe_states.line_follower_state import LineFollowerState
from hector_flexbe_states.move_to_waypoint_state import MoveToWaypointState as hector_flexbe_states__MoveToWaypointState
from hector_flexbe_states.add_waypoint_to_array_state import AddWaypointToArrayState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PointStamped
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
		self.add_parameter('speed', 0.5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1012 y:445, x:1229 y:485
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pose'])
		_state_machine.userdata.speed = self.speed
		_state_machine.userdata.camera_topic = '/front_rgbd_cam/color/image_rect_color'
		_state_machine.userdata.drive_backwards = False
		_state_machine.userdata.nextWaypoint = ''
		_state_machine.userdata.waypoints = []
		_state_machine.userdata.pose = PointStamped()
		_state_machine.userdata.first_call = True

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:749 y:556, x:727 y:378, x:727 y:64, x:721 y:222, x:378 y:477, x:519 y:498, x:718 y:127, x:719 y:296
		_sm_findline_0 = ConcurrencyContainer(outcomes=['failed', 'line_found', 'waypoint_reached'], input_keys=['nextWaypoint', 'speed', 'drive_backwards', 'camera_topic', 'first_call'], output_keys=['first_call'], conditions=[
										('failed', [('MoveToWaypoint', 'failed')]),
										('waypoint_reached', [('MoveToWaypoint', 'reached')]),
										('failed', [('SearchLine', 'failed')]),
										('failed', [('MoveToWaypoint', 'stuck')]),
										('line_found', [('SearchLine', 'reached')])
										])

		with _sm_findline_0:
			# x:232 y:84
			OperatableStateMachine.add('SearchLine',
										LineFollowerState(timeout_sec=self.timeout_sec, search_line=True),
										transitions={'failed': 'failed', 'reached': 'line_found'},
										autonomy={'failed': Autonomy.Off, 'reached': Autonomy.Off},
										remapping={'camera_topic': 'camera_topic', 'drive_backwards': 'drive_backwards', 'speed': 'speed'})

			# x:226 y:241
			OperatableStateMachine.add('MoveToWaypoint',
										hector_flexbe_states__MoveToWaypointState(position_tolerance=0.2, angle_tolerance=3, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=True),
										transitions={'reached': 'waypoint_reached', 'failed': 'failed', 'stuck': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'waypoint': 'nextWaypoint', 'speed': 'speed', 'first_call': 'first_call'})


		# x:837 y:248, x:843 y:379
		_sm_followline_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['nextWaypoint', 'drive_backwards', 'camera_topic', 'speed', 'first_call'], output_keys=['first_call'])

		with _sm_followline_1:
			# x:82 y:178
			OperatableStateMachine.add('hasWaypoint?',
										DecisionState(outcomes=['waypoint', 'no_waypoint'], conditions=lambda x: 'waypoint' if x else 'no_waypoint'),
										transitions={'waypoint': 'FindLine', 'no_waypoint': 'LineFollower_noWaypoint'},
										autonomy={'waypoint': Autonomy.Off, 'no_waypoint': Autonomy.Off},
										remapping={'input_value': 'nextWaypoint'})

			# x:331 y:175
			OperatableStateMachine.add('FindLine',
										_sm_findline_0,
										transitions={'failed': 'FindLine', 'line_found': 'LineFollower', 'waypoint_reached': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'line_found': Autonomy.Inherit, 'waypoint_reached': Autonomy.Inherit},
										remapping={'nextWaypoint': 'nextWaypoint', 'speed': 'speed', 'drive_backwards': 'drive_backwards', 'camera_topic': 'camera_topic', 'first_call': 'first_call'})

			# x:281 y:370
			OperatableStateMachine.add('LineFollower_noWaypoint',
										LineFollowerState(timeout_sec=self.timeout_sec, search_line=False),
										transitions={'failed': 'failed', 'reached': 'finished'},
										autonomy={'failed': Autonomy.Off, 'reached': Autonomy.Off},
										remapping={'camera_topic': 'camera_topic', 'drive_backwards': 'drive_backwards', 'speed': 'speed'})

			# x:326 y:37
			OperatableStateMachine.add('LineFollower',
										LineFollowerState(timeout_sec=10, search_line=False),
										transitions={'failed': 'FindLine', 'reached': 'FindLine'},
										autonomy={'failed': Autonomy.Off, 'reached': Autonomy.Off},
										remapping={'camera_topic': 'camera_topic', 'drive_backwards': 'drive_backwards', 'speed': 'speed'})



		with _state_machine:
			# x:114 y:44
			OperatableStateMachine.add('getWaypoints',
										flexbe_argos_states__GetPathFromServiceState(service_topic='/path_to_follow'),
										transitions={'succeeded': 'add_waypoint', 'failed': 'haveWaypoints?'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints2': 'waypoints'})

			# x:1342 y:62
			OperatableStateMachine.add('SwitchDirection',
										SwitchLineFollowerDirectionState(camera_topic_forwards='/front_rgbd_cam/color/image_rect_color', camera_topic_backwards='/back_rgbd_cam/color/image_rect_color'),
										transitions={'finished': 'reachedLower'},
										autonomy={'finished': Autonomy.Off},
										remapping={'reverse': 'drive_backwards', 'camera_topic': 'camera_topic'})

			# x:369 y:58
			OperatableStateMachine.add('haveWaypoints?',
										DecisionState(outcomes=['waypoints', 'no_waypoints'], conditions=lambda x: 'waypoints' if x.value.poses else 'no_waypoints'),
										transitions={'waypoints': 'getCurrentWaypoint', 'no_waypoints': 'FollowLine'},
										autonomy={'waypoints': Autonomy.Off, 'no_waypoints': Autonomy.Off},
										remapping={'input_value': 'waypoints'})

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

			# x:569 y:226
			OperatableStateMachine.add('getCurrentWaypoint',
										GetWaypointFromArrayState(position=0),
										transitions={'succeeded': 'FollowLine', 'empty': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoint': 'nextWaypoint'})

			# x:836 y:168
			OperatableStateMachine.add('FollowLine',
										_sm_followline_1,
										transitions={'finished': 'write_3d_map', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'nextWaypoint': 'nextWaypoint', 'drive_backwards': 'drive_backwards', 'camera_topic': 'camera_topic', 'speed': 'speed', 'first_call': 'first_call'})

			# x:209 y:180
			OperatableStateMachine.add('add_waypoint',
										AddWaypointToArrayState(),
										transitions={'succeeded': 'haveWaypoints?', 'failed': 'haveWaypoints?'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pose', 'waypoints': 'waypoints'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
