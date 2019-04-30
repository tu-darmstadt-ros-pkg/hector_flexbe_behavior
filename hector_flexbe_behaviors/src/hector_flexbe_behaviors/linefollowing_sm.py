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
from hector_flexbe_states.move_to_waypoint_state import MoveToWaypointState as hector_flexbe_states__MoveToWaypointState
from hector_flexbe_states.line_follower_state import LineFollowerState
from hector_flexbe_states.find_line_state import FindLineState
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
		self.add_parameter('distance_to_line', 1.4)
		self.add_parameter('speed', 0.2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:661 y:153, x:822 y:461
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.speed = 0.2
		_state_machine.userdata.camera_topic = '/front_rgbd_cam/color/image_rect_color'
		_state_machine.userdata.drive_backwards = False
		_state_machine.userdata.nextWaypoint = ''
		_state_machine.userdata.waypoints = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:84 y:26
			OperatableStateMachine.add('getWaypoints',
										GetPathState(pathTopic='/path_to_follow'),
										transitions={'succeeded': 'getCurrentWaypoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints2': 'waypoints'})

			# x:311 y:30
			OperatableStateMachine.add('getCurrentWaypoint',
										GetWaypointFromArrayState(position=0),
										transitions={'succeeded': 'FindLine', 'empty': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'empty': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoint': 'nextWaypoint'})

			# x:674 y:27
			OperatableStateMachine.add('SwitchDirection',
										SwitchLineFollowerDirectionState(camera_topic_forwards='/front_rgbd_cam/color/image_rect_color', camera_topic_backwards='/back_rgbd_cam/color/image_rect_color'),
										transitions={'finished': 'getCurrentWaypoint'},
										autonomy={'finished': Autonomy.Off},
										remapping={'reverse': 'drive_backwards', 'camera_topic': 'camera_topic'})

			# x:1038 y:27
			OperatableStateMachine.add('MoveToWaypoint',
										hector_flexbe_states__MoveToWaypointState(desired_speed=self.speed, position_tolerance=0.2, angle_tolerance=3, rotate_to_goal=0, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=False),
										transitions={'reached': 'SwitchDirection', 'failed': 'failed', 'stuck': 'MoveToWaypoint'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'waypoint': 'nextWaypoint'})

			# x:759 y:191
			OperatableStateMachine.add('LineFollower',
										LineFollowerState(timeout_sec=self.timeout_sec, speed=self.speed),
										transitions={'reached': 'MoveToWaypoint', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'camera_topic': 'camera_topic', 'drive_backwards': 'drive_backwards'})

			# x:550 y:217
			OperatableStateMachine.add('FindLine',
										FindLineState(speed=self.speed, distance=self.distance_to_line, velocity_topic='/cmd_vel'),
										transitions={'reached': 'LineFollower'},
										autonomy={'reached': Autonomy.Off},
										remapping={'reverse': 'drive_backwards'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
